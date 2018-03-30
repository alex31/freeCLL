#include "castel_link.hpp"

#include <ch.h>
#include <hal.h>
#include <algorithm>
#include <cmath>
#include "globalVar.h"
#include "stdutil.h"
#include "config.hpp"
#include "simpleSerialMessage.h"


/*
  TODO : 

 


 */


/*
#                     _            __   _            _    _      _                          
#                    | |          / _| (_)          (_)  | |    (_)                         
#                  __| |    ___  | |_   _    _ __    _   | |_    _     ___    _ __          
#                 / _` |   / _ \ |  _| | |  | '_ \  | |  | __|  | |   / _ \  | '_ \         
#                | (_| |  |  __/ | |   | |  | | | | | |  \ |_   | |  | (_) | | | | |        
#                 \__,_|   \___| |_|   |_|  |_| |_| |_|   \__|  |_|   \___/  |_| |_|        
*/


typedef enum		 {WAIT_FOR_PULSE, PULSE_ACQUIRED} PulseSate;


class LinkState
{
public:
  LinkState(PWMDriver &_pwmd, ICUDriver &_icud,
	    const uint32_t _pwmCmdCh, const uint32_t _pwmCmdHiZ) :
    currentRaw{nullptr}, pulseState{WAIT_FOR_PULSE}, pwmd{_pwmd}, icud(_icud),
    pwmCmdCh{_pwmCmdCh}, pwmCmdHiZ{_pwmCmdHiZ}, escIdx {indexer++} {};

  void initFifoFetch(void);
  void pwmStartIcu_cb(void);
  void pwmModeHiZ_cb(void);
  void pwmModePushpull_cb(void);
  void icuWidth_cb(void);
  void setDuty(int16_t dutyPerTenThousand);
  int16_t getDuty(void) const {return currentDuty;};
  void setDutyFromISR(int16_t dutyPerTenThousand);
  
private:
  static uint8_t indexer;

  castelLinkRawData * volatile currentRaw;
  volatile PulseSate  pulseState;
  PWMDriver &pwmd;
  ICUDriver &icud;
  const uint32_t    pwmCmdCh;
  const uint32_t    pwmCmdHiZ;
  volatile pwmcnt_t currentDuty;
  uint8_t	    escIdx;
};


/*
#                 _ __                   _              _      _   _    _ __                 
#                | '_ \                 | |            | |    | | | |  | '_ \                
#                | |_) |  _ __    ___   | |_     ___   | |_   | |_| |  | |_) |   ___         
#                | .__/  | '__|  / _ \  | __|   / _ \  | __|   \__, |  | .__/   / _ \        
#                | |     | |    | (_) | \ |_   | (_) | \ |_     __/ |  | |     |  __/        
#                |_|     |_|     \___/   \__|   \___/   \__|   |___/   |_|      \___|        
*/
static void sendTelemetryThd (void *arg);
static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData);

/*
#                  __ _   _            _               _          
#                 / _` | | |          | |             | |         
#                | (_| | | |    ___   | |__     __ _  | |         
#                 \__, | | |   / _ \  | '_ \   / _` | | |         
#                  __/ | | |  | (_) | | |_) | | (_| | | |         
#                 |___/  |_|   \___/  |_.__/   \__,_| |_|         
*/
static THD_WORKING_AREA(waSendTelemetry, 1024);
static castelLinkRawData raw_pool[CASTELLINK::FIFO_SIZE];
static msg_t msg_raw_fifo[CASTELLINK::FIFO_SIZE];
static objects_fifo_t raw_fifo;
static virtual_timer_t vtTelemetry;

// helper function to help debug with Logic Analyser
static inline void debugPulse (const ioline_t line) {
#ifdef DEBUG_ASSERTS_ENABLED
  palSetLine(line);
  chSysPolledDelayX(TIME_US2I(1));
  palClearLine(line);
#else
  //  (void) line;
#endif
}



static std::array<LinkState, 2>  escLinks {
  LinkState{CASTELLINK::PWM, CASTELLINK::ICU1,
      CASTELLINK::PWM_COMMAND_CH_1, CASTELLINK::PWM_HIGHZ_CH_1},
  LinkState{CASTELLINK::PWM, CASTELLINK::ICU2,
      CASTELLINK::PWM_COMMAND_CH_2, CASTELLINK::PWM_HIGHZ_CH_2}
};

/*
#                                         __   _     __ _         
#                                        / _| (_)   / _` |        
#                  ___    ___    _ __   | |_   _   | (_| |        
#                 / __|  / _ \  | '_ \  |  _| | |   \__, |        
#                | (__  | (_) | | | | | | |   | |    __/ |        
#                 \___|  \___/  |_| |_| |_|   |_|   |___/         
*/

// serial link with computer host
static constexpr SerialConfig  hostcfg =  {
  .speed = CASTELLINK::TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN, // 1 bit de stop, detection d'erreur de trame avancée
  .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
};


/*
° pwm used to drive 2 ESCs : 4 channels are used, 2 by ESC
  foreach ESC, one channel for pwm generation, one channel to trig high impedance mode for pwm when
  castel creation ESC drive the line

° static configuration values are in user_config.hpp

° start of pwm (common to all pwm since there are left aligned) ISR do :
  + start input capture for get ESC telemetry
  + start a timer to stop input capture and revert LINE from HiZ

° pwm channel HiZ ISR set pwm channel in HiZ mode after the PWm signal as been pulled to high quickly by transistor
  then only it is changed to HiZ

*/
static constexpr PWMConfig pwmcfg = {     
  .frequency = CASTELLINK::TICK_FREQ,
  .period    = CASTELLINK::TICK_PER_PERIOD,
  .callback  = [] (PWMDriver *pwmd) {
    (void) pwmd;
    chSysLockFromISR();
    escLinks[0].pwmStartIcu_cb();
    escLinks[1].pwmStartIcu_cb();
    gptStopTimerI(&CASTELLINK::GPT_PUSHPULL);
    gptStartOneShotI(&CASTELLINK::GPT_PUSHPULL, CASTELLINK::PUSHPULL_TIMESHIFT_TICKS);
    chSysUnlockFromISR();
  },
  .channels  = {
    [CASTELLINK::PWM_COMMAND_CH_1] =
    {.mode = PWM_OUTPUT_ACTIVE_LOW, .callback = nullptr}, 

    [CASTELLINK::PWM_HIGHZ_CH_1] =
    {.mode = PWM_OUTPUT_DISABLED,   .callback = [] (PWMDriver *pwmd) {
	(void) pwmd;
	chSysLockFromISR();
	escLinks[0].pwmModeHiZ_cb();
	chSysUnlockFromISR();
      }},

    
    [CASTELLINK::PWM_HIGHZ_CH_2] =
    {.mode = PWM_OUTPUT_DISABLED,   .callback = [] (PWMDriver *pwmd) {
	(void) pwmd;
	chSysLockFromISR();
	escLinks[1].pwmModeHiZ_cb();
	chSysUnlockFromISR();
      }},
    
    [CASTELLINK::PWM_COMMAND_CH_2] =
    {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = nullptr},

    [4] = {.mode = PWM_OUTPUT_DISABLED,   .callback = nullptr},
    [5] = {.mode = PWM_OUTPUT_DISABLED,   .callback = nullptr}
  },
  .cr2  = 0,
  .dier = 0
};

/*
  each ESC channel need a timer in ICU mode
  after each period ISR, pulse is registered in a data structure
  when data structure is full, message is sent so a thread which convert raw data
  to decoded data, and theses datas are sent over serial link
 */
static constexpr ICUConfig icu1cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = CASTELLINK::ICU_TIMFREQ,
  .width_cb = [] (ICUDriver *icud) {
    (void) icud;
    chSysLockFromISR();
    escLinks[0].icuWidth_cb();
    chSysUnlockFromISR();
  },
  .period_cb = nullptr, 
  .overflow_cb = nullptr,
  .channel = CASTELLINK::ICU1_CHANNEL,
  .dier = 0,
};

static constexpr ICUConfig icu2cfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = CASTELLINK::ICU_TIMFREQ,
  .width_cb = [] (ICUDriver *icud) {
    (void) icud;
    chSysLockFromISR();
    escLinks[1].icuWidth_cb();
    chSysUnlockFromISR();
  },
  .period_cb = nullptr, 
  .overflow_cb = nullptr,
  .channel = CASTELLINK::ICU2_CHANNEL,
  .dier = 0,
};


/*
  after a fixed time pwm channels which have bee poreviously passed in HiZ mode are returned in
  pushpush mode
 */
static constexpr GPTConfig gptcfg = {
  CASTELLINK::GPT_PP_FREQ,
  [] (GPTDriver *gptp) {
    (void) gptp;
    chSysLockFromISR();
    escLinks[0].pwmModePushpull_cb();
    escLinks[1].pwmModePushpull_cb();
    chSysUnlockFromISR();
  },
  0,
  0
};


 /*
#                 _ __            _       _    _                 
#                | '_ \          | |     | |  (_)                
#                | |_) |  _   _  | |__   | |   _     ___         
#                | .__/  | | | | | '_ \  | |  | |   / __|        
#                | |     | |_| | | |_) | | |  | |  | (__         
#                |_|      \__,_| |_.__/  |_|  |_|   \___|        
#                         _ __    _          
#                        | '_ \  (_)         
#                  __ _  | |_) |  _          
#                 / _` | | .__/  | |         
#                | (_| | | |     | |         
#                 \__,_| |_|     |_|         
*/



void castelLinkStart(void)
{
  chVTObjectInit(&vtTelemetry);
  chFifoObjectInit (&raw_fifo, sizeof(castelLinkRawData), CASTELLINK::FIFO_SIZE,
		    4,  raw_pool, msg_raw_fifo);
  chThdCreateStatic(waSendTelemetry, sizeof(waSendTelemetry), NORMALPRIO, &sendTelemetryThd, NULL);


  escLinks[0].initFifoFetch();
  escLinks[1].initFifoFetch();

  gptStart(&CASTELLINK::GPT_PUSHPULL, &gptcfg);
  pwmStart(&CASTELLINK::PWM, &pwmcfg);
  icuStart(&CASTELLINK::ICU1, &icu1cfg);
  icuOptStart(&CASTELLINK::ICU2, &icu2cfg); // TIM15 cannot be start with official ChibiOS function
  
  sdStart(&CASTELLINK::SD_TELEMETRY, &hostcfg);

  simpleMsgBind (CASTELLINK::STREAM_TELEMETRY_PTR, telemetryReceive_cb, nullptr);
}


void castelLinkSetDuty(const uint8_t escIdx, const int16_t dutyPerTenThousand)
{
  if (escIdx < escLinks.size()) {
    escLinks[escIdx].setDuty(dutyPerTenThousand);
  }

  //debugPulse(LINE_DBG_LINEA01);
  if constexpr (CASTELLINK::SHUTDOWN_WITHOUT_TELEMETRY_MS != 0) {
#if DEBUG_ASSERTS_ENABLED == FALSE
      // should no be needed, perhaps BUGS here :
      // works without chVTReset in all configurations but this one :
      // compile with -Ofast and  CH_DBG_ENABLE_ASSERTS = FALSE
      // chVTReset is set here when  DEBUG_ASSERTS_ENABLED == FALSE as a work around, not a fix
      chVTReset(&vtTelemetry);
#endif
      // if serial link do not send any data for more than SHUTDOWN_WITHOUT_TELEMETRY_MS
      // we shut down pwm (engine stop)
      chVTSet(&vtTelemetry, TIME_MS2I(CASTELLINK::SHUTDOWN_WITHOUT_TELEMETRY_MS),
	      [] (void *arg) {(void) arg;
		chSysLockFromISR();
		//		debugPulse(LINE_DBG_LINEA06);
		escLinks[0].setDutyFromISR(CASTELLINK::PWM_DISABLE);
		escLinks[1].setDutyFromISR(CASTELLINK::PWM_DISABLE);
		chSysUnlockFromISR();
	      },
	      nullptr);
    }
}



/*
#                        _                                
#                       | |                               
#                  ___  | |    __ _   ___    ___          
#                 / __| | |   / _` | / __|  / __|         
#                | (__  | |  | (_| | \__ \  \__ \         
#                 \___| |_|   \__,_| |___/  |___/         
#                     _            __   _            _    _      _                          
#                    | |          / _| (_)          (_)  | |    (_)                         
#                  __| |    ___  | |_   _    _ __    _   | |_    _     ___    _ __          
#                 / _` |   / _ \ |  _| | |  | '_ \  | |  | __|  | |   / _ \  | '_ \         
#                | (_| |  |  __/ | |   | |  | | | | | |  \ |_   | |  | (_) | | | | |        
#                 \__,_|   \___| |_|   |_|  |_| |_| |_|   \__|  |_|   \___/  |_| |_|        
*/



/*
#                                       _             _    _        _            _            
#                                      | |           | |  | |      (_)          | |           
#                  ___    __ _   ___   | |_     ___  | |  | |       _    _ __   | | _         
#                 / __|  / _` | / __|  | __|   / _ \ | |  | |      | |  | '_ \  | |/ /        
#                | (__  | (_| | \__ \  \ |_   |  __/ | |  | |____  | |  | | | | |   <         
#                 \___|  \__,_| |___/   \__|   \___| |_|  |______| |_|  |_| |_| |_|\_\        
#                 _____                       _____             _                    
#                |  __ \                     |  __ \           | |                   
#                | |__) |   __ _  __      __ | |  | |    __ _  | |_     __ _         
#                |  _  /   / _` | \ \ /\ / / | |  | |   / _` | | __|   / _` |        
#                | | \ \  | (_| |  \ V  V /  | |__| |  | (_| | \ |_   | (_| |        
#                |_|  \_\  \__,_|   \_/\_/   |_____/    \__,_|  \__|   \__,_|        
*/
castelLinkRawData::castelLinkRawData() : index{0}, raw{0} 
{
}

void castelLinkRawData::resetIndex(void)
{
  index = 0;
}


bool  castelLinkRawData::push(const uint16_t val)
{
  if (index < raw_len) {
    raw[index++]=val;
  }

  const bool filled = (index == raw_len);

  return filled;
}

void castelLinkRawData::dbgTrace(void) const
{
#if defined TRACE
  DebugTrace ("");
  for (const auto& elem : raw) {
    chprintf (STREAM_SHELL_PTR, "%u, ", elem);
  }
  DebugTrace ("");
#endif
}

/*
#                                       _             _    _        _            _            
#                                      | |           | |  | |      (_)          | |           
#                  ___    __ _   ___   | |_     ___  | |  | |       _    _ __   | | _         
#                 / __|  / _` | / __|  | __|   / _ \ | |  | |      | |  | '_ \  | |/ /        
#                | (__  | (_| | \__ \  \ |_   |  __/ | |  | |____  | |  | | | | |   <         
#                 \___|  \__,_| |___/   \__|   \___| |_|  |______| |_|  |_| |_| |_|\_\        
#                 _____             _                    
#                |  __ \           | |                   
#                | |  | |    __ _  | |_     __ _         
#                | |  | |   / _` | | __|   / _` |        
#                | |__| |  | (_| | \ |_   | (_| |        
#                |_____/    \__,_|  \__|   \__,_|        
*/
castelLinkData::castelLinkData() : datas{0},
				   escIdx{0},
				   raw{nullptr}
{
}

castelLinkData::castelLinkData(const castelLinkRawData* _raw) : 
							 raw{_raw}
{
  convertValues();
}


void castelLinkData::populate(const castelLinkRawData* _raw)
{
  raw = _raw;
  convertValues();
}

void castelLinkData::dbgTrace(void) const
{
#if defined TRACE
  DebugTrace ("bat = %.2f ripple=%.2f current=%.2f throttle=%.2f power=%.2f",
	      bat_voltage, ripple_voltage, current, throttle, power);
  DebugTrace ("rpm=%.2f bec_v=%.2f bec_c=%.2f temp=%.2f",
	      rpm, bec_voltage, bec_current, temperature);
#endif	      
}


void castelLinkData::convertValues(void) 
{
  if (raw == nullptr)
    chSysHalt ("convertValues : raw  == nullptr");

  escIdx = raw->getEscIdx();
  
  const float cal_coeff_0 = std::min(raw->get_temp_linear_or_cal(), raw->get_temp_ntc_or_cal());
  const float cal_coeff_1 = raw->get_calibration_1ms();
  const bool temp_NTC = raw->get_temp_linear_or_cal() < raw->get_temp_ntc_or_cal();

  for (size_t i=0; i< datas.size(); i++) {
    if ((i+1 < raw->get_raw_len()) && (i < scale_coeffs.size()))
      datas[i] = ((raw->get_raw_ref()[i+1] - cal_coeff_0) / cal_coeff_1) * scale_coeffs[i];
  }


  /*
    ° attempt to make another correction between pwm pulse len and ESC measured pulse len
    ° does not seem to bring anything good
   */
  // const float cal_coeff_2 = escLinks[raw->getEscIdx()].getDuty() / (throttle * 1000.0f);
  // for (auto &v : datas) {
  //   v *= cal_coeff_2;
  // }
  
  if (temp_NTC == true) {
    const float val = ((raw->get_temp_ntc_or_cal() - cal_coeff_0) / cal_coeff_1) * scale_coeffs[9];
    const float r0 = 10000;
    const float r2 = 10200;
    const float b = 3455;
    // from castel link live protocol documentation
    temperature = 1.0 / (logf(val*r2/(255.0-val)/r0) / b + 1/298.0) - 273;
  }
}

void castelLinkData::sendTelemetry(void) 
{
  simpleMsgSend(CASTELLINK::STREAM_TELEMETRY_PTR, reinterpret_cast<uint8_t *> (this),
		sizeof(datas) + sizeof(escIdx));
}

uint8_t  LinkState::indexer = 0;

void LinkState::initFifoFetch(void)
{
  currentRaw =  static_cast<castelLinkRawData *> (chFifoTakeObjectTimeout(&raw_fifo, TIME_IMMEDIATE));
  currentRaw->setEscIdx (escIdx);
}

void LinkState::pwmStartIcu_cb(void)
{
  pulseState =  WAIT_FOR_PULSE;

  //  debugPulse(LINE_DBG_LINEA01);
  icuStartCaptureI(&icud);
  icuEnableNotificationsI(&icud);
}

void LinkState::pwmModeHiZ_cb(void)
{
  pwmMaskChannelSide(&pwmd, pwmCmdCh, PWM_NORMAL, true);
}

void LinkState::pwmModePushpull_cb(void)
{
  pwmMaskChannelSide(&pwmd, pwmCmdCh, PWM_NORMAL, false);
  icuStopCaptureI(&icud);
  if ((pulseState ==  WAIT_FOR_PULSE) && currentRaw) {
    currentRaw->resetIndex();
  }
}


void LinkState::icuWidth_cb(void)
{
  const icucnt_t width = icuGetWidthX(&icud);
  pulseState = PULSE_ACQUIRED;
  
  if (currentRaw) {
    if (width >= CASTELLINK::ICU_MINPULSE_TICK && width <= CASTELLINK::ICU_MAXPULSE_TICK) {
      icuStopCaptureI(&icud);
      if (currentRaw->push(width)) {
	chFifoSendObjectI(&raw_fifo, currentRaw);
	currentRaw =  static_cast<castelLinkRawData *> (chFifoTakeObjectI(&raw_fifo));
	currentRaw->setEscIdx(escIdx);
      }
    }
  } else { // currentRaw == nullptr
    currentRaw = static_cast<castelLinkRawData *> (chFifoTakeObjectI(&raw_fifo));
    currentRaw->setEscIdx(escIdx);
  }
}


void LinkState::setDuty(int16_t castelDuty)
{
  currentDuty = castelDuty;
  
  if (castelDuty != CASTELLINK::PWM_DISABLE) {
    pwmEnableChannel(&pwmd, pwmCmdCh, castelDuty);
    pwmEnableChannel(&pwmd, pwmCmdHiZ,
		     castelDuty + CASTELLINK::HIGHZ_TIMESHIFT_TICKS);
     
    pwmEnablePeriodicNotification(&pwmd);
    pwmEnableChannelNotification(&pwmd, pwmCmdHiZ);
  } else {
    pwmDisableChannel(&pwmd, pwmCmdCh);
    pwmDisableChannel(&pwmd, pwmCmdHiZ);
  }
}
  
void LinkState::setDutyFromISR(int16_t castelDuty)
{
  currentDuty = castelDuty;
  
  if (castelDuty != CASTELLINK::PWM_DISABLE) {
    pwmEnableChannelI(&pwmd, pwmCmdCh, castelDuty);
    pwmEnableChannelI(&pwmd, pwmCmdHiZ,
		     castelDuty + CASTELLINK::HIGHZ_TIMESHIFT_TICKS);
   
    
    pwmEnablePeriodicNotificationI(&pwmd);
    pwmEnableChannelNotificationI(&pwmd, pwmCmdHiZ);
  } else {
    pwmDisableChannelI(&pwmd, pwmCmdCh);
    pwmDisableChannelI(&pwmd, pwmCmdHiZ);
  }
}



/*
#                                _    _    _                      _            
#                               | |  | |  | |                    | |           
#                  ___    __ _  | |  | |  | |__     __ _    ___  | | _         
#                 / __|  / _` | | |  | |  | '_ \   / _` |  / __| | |/ /        
#                | (__  | (_| | | |  | |  | |_) | | (_| | | (__  |   <         
#                 \___|  \__,_| |_|  |_|  |_.__/   \__,_|  \___| |_|\_\        
*/


static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData)
{
  (void) userData;

  

  if (len != sizeof(TelemetryDownMsg)) {
    DebugTrace ("Msg len error : rec %u instead of waited %u", len, sizeof(TelemetryDownMsg));
  } else {
    const TelemetryDownMsg *msg = reinterpret_cast<const TelemetryDownMsg *> (buffer);
    switch (msg->msgId) {
    case PWM_ORDER :
      castelLinkSetDuty(msg->escIdx, msg->duty);
      break;
    case CALIBRATE : DebugTrace ("Calibrate not yet implemented");
      break;
    }
  }
}


/*
#                 _      _                                 _                 
#                | |    | |                               | |                
#                | |_   | |__    _ __    ___    __ _    __| |   ___          
#                | __|  | '_ \  | '__|  / _ \  / _` |  / _` |  / __|         
#                \ |_   | | | | | |    |  __/ | (_| | | (_| |  \__ \         
#                 \__|  |_| |_| |_|     \___|  \__,_|  \__,_|  |___/         
*/

static void sendTelemetryThd (void *arg)
{
  (void)arg;
  chRegSetThreadName("telemetry");
  castelLinkRawData *rawData=nullptr;
  
  while (true) {
    chFifoReceiveObjectTimeout(&raw_fifo, reinterpret_cast<void **> (&rawData),  TIME_INFINITE);
    castelLinkData processedData(rawData);
    processedData.sendTelemetry();
    //    rawData->dbgTrace();
    processedData.dbgTrace();
    chFifoReturnObject(&raw_fifo, rawData);
  }
}


