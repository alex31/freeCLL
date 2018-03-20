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
#                     _            __   _            _    _      _                          
#                    | |          / _| (_)          (_)  | |    (_)                         
#                  __| |    ___  | |_   _    _ __    _   | |_    _     ___    _ __          
#                 / _` |   / _ \ |  _| | |  | '_ \  | |  | __|  | |   / _ \  | '_ \         
#                | (_| |  |  __/ | |   | |  | | | | | |  \ |_   | |  | (_) | | | | |        
#                 \__,_|   \___| |_|   |_|  |_| |_| |_|   \__|  |_|   \___/  |_| |_|        
*/


typedef enum  : uint16_t {PWM_ORDER=0, CALIBRATE} MessageId;


typedef struct {
  MessageId msgId;
  uint16_t  value[2];
} TelemetryDownMsg;

/*
#                 _ __                   _              _      _   _    _ __                 
#                | '_ \                 | |            | |    | | | |  | '_ \                
#                | |_) |  _ __    ___   | |_     ___   | |_   | |_| |  | |_) |   ___         
#                | .__/  | '__|  / _ \  | __|   / _ \  | __|   \__, |  | .__/   / _ \        
#                | |     | |    | (_) | \ |_   | (_) | \ |_     __/ |  | |     |  __/        
#                |_|     |_|     \___/   \__|   \___/   \__|   |___/   |_|      \___|        
*/
static void pwmStartIcu_cb (PWMDriver *pwmd);
static void pwmModeHighZ_cb (PWMDriver *pwmd);
static void pwmModePushpull_cb (PWMDriver *pwmd);
static void icuWidth_cb (ICUDriver *icud);
static void icuTimout_cb (ICUDriver *icud);
static void sendTelemetryThd (void *arg);
static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData);
static void initPulse_cb(PWMDriver *pwmp);
static void gpt6_cb(GPTDriver *gptp);
static void gpt7_cb(GPTDriver *gptp);
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
static  castelLinkRawData * volatile currentRaw = nullptr;


volatile bool icuOverflow = false;

/*
#                                         __   _     __ _         
#                                        / _| (_)   / _` |        
#                  ___    ___    _ __   | |_   _   | (_| |        
#                 / __|  / _ \  | '_ \  |  _| | |   \__, |        
#                | (__  | (_) | | | | | | |   | |    __/ |        
#                 \___|  \___/  |_| |_| |_|   |_|   |___/         
*/




static constexpr SerialConfig  hostcfg =  {
  .speed = CASTELLINK::TELEMETRY_BAUD,
  .cr1 = 0,                                      // pas de parité
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN, // 1 bit de stop, detection d'erreur de trame avancée
  .cr3 = 0                                       // pas de controle de flux hardware (CTS, RTS)
};



static constexpr PWMConfig pwmcfg = {     
  .frequency = CASTELLINK::TICK_FREQ,
  .period    = CASTELLINK::TICK_PER_PERIOD,
  .callback  = &pwmStartIcu_cb,
  .channels  = {
    {.mode = PWM_OUTPUT_ACTIVE_LOW, .callback = &initPulse_cb}, // start pulse
    {.mode = PWM_OUTPUT_DISABLED,   .callback = &pwmModeHighZ_cb},
    {.mode = PWM_OUTPUT_DISABLED,   .callback = &pwmModePushpull_cb},
    {.mode = PWM_OUTPUT_DISABLED,   .callback = nullptr},
    {.mode = PWM_OUTPUT_DISABLED,   .callback = nullptr},
    {.mode = PWM_OUTPUT_DISABLED,   .callback = nullptr}
  },
  .cr2  = 0,
  .dier = 0
};


static constexpr ICUConfig icucfg = {
  .mode = ICU_INPUT_ACTIVE_HIGH,
  .frequency = CASTELLINK::ICU_TIMFREQ,
  .width_cb = &icuWidth_cb, 
  .period_cb = nullptr, 
  .overflow_cb = icuTimout_cb,
  .channel = CASTELLINK::ICU_CHANNEL,
  .dier = 0,
};


/*
 * GPT6 configuration.
 */
static constexpr GPTConfig gpt6cfg = {
  static_cast<uint32_t>(1e6),    /* 1Mhz timer clock.  1000 ticks pour 1ms  */
  &gpt6_cb,   
  0,
  0
};

/*
 * GPT7 configuration.
 */
static constexpr GPTConfig gpt7cfg = {
  static_cast<uint32_t>(1e6),    /* 1Mhz timer clock.  1000 ticks pour 1ms  */
  &gpt7_cb,   
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
  chFifoObjectInit (&raw_fifo, sizeof(castelLinkRawData), CASTELLINK::FIFO_SIZE,
		    4,  raw_pool, msg_raw_fifo);
  chThdCreateStatic(waSendTelemetry, sizeof(waSendTelemetry), NORMALPRIO, &sendTelemetryThd, NULL);
  
  currentRaw = static_cast<castelLinkRawData *> (chFifoTakeObjectTimeout(&raw_fifo, TIME_IMMEDIATE));


  gptStart(&GPTD6, &gpt6cfg);
  gptStart(&GPTD7, &gpt7cfg);
  pwmStart(&CASTELLINK::PWM, &pwmcfg);
  icuStart(&CASTELLINK::ICU, &icucfg);
  sdStart(&CASTELLINK::SD_TELEMETRY, &hostcfg);

  simpleMsgBind (CASTELLINK::STREAM_TELEMETRY_PTR, telemetryReceive_cb, nullptr);
}


void castelLinkSetDuty(uint16_t dutyPerTenThousand)
{
  const pwmcnt_t castelDuty = PWM_PERCENTAGE_TO_WIDTH(&CASTELLINK::PWM, dutyPerTenThousand);

  if (dutyPerTenThousand != 0) {
    pwmEnableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL, castelDuty);
    pwmEnableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_HIGHZ_CHANNEL,
		     castelDuty + CASTELLINK::HIGHZ_TIMESHIFT_TICKS);
    pwmEnableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_PUSHPULL_CHANNEL,
		     CASTELLINK::PUSHPULL_DUTY_TICKS);

    pwmEnablePeriodicNotification(&CASTELLINK::PWM);
    pwmEnableChannelNotification(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL);
    pwmEnableChannelNotification(&CASTELLINK::PWM, CASTELLINK::PWM_HIGHZ_CHANNEL);
    pwmEnableChannelNotification(&CASTELLINK::PWM, CASTELLINK::PWM_PUSHPULL_CHANNEL);
  } else {
    pwmDisableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL);
    pwmDisableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_HIGHZ_CHANNEL);
    pwmDisableChannel(&CASTELLINK::PWM, CASTELLINK::PWM_PUSHPULL_CHANNEL);
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
  DebugTrace ("");
  for (const auto& elem : raw) {
    chprintf (chp, "%u, ", elem);
  }
  DebugTrace ("");
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
castelLinkData::castelLinkData() : raw{nullptr},
				   bat_voltage{0},
				   ripple_voltage{0},
				   current{0},
				   throttle{0},
				   power{0},
				   rpm{0},
				   bec_voltage{0},
				   bec_current{0},
				   temperature{0},
				   channel{0}
				   
{
}

castelLinkData::castelLinkData(const castelLinkRawData* _raw,
			       const uint8_t _channel) : raw{_raw},
							bat_voltage{0},
							ripple_voltage{0},
							current{0},
							throttle{0},
							power{0},
							rpm{0},
							bec_voltage{0},
							bec_current{0},
							temperature{0},
							channel{_channel}
							
{
  convertValues();
}


void castelLinkData::populate(const castelLinkRawData* _raw, const uint8_t _channel)
{
  raw = _raw;
  channel = _channel;
  convertValues();
}

void castelLinkData::dbgTrace(void) const
{
  DebugTrace ("bat = %.2f ripple=%.2f current=%.2f throttle=%.2f power=%.2f",
	      bat_voltage, ripple_voltage, current, throttle, power);
  DebugTrace ("rpm=%.2f bec_v=%.2f bec_c=%.2f temp=%.2f",
	      rpm, bec_voltage, bec_current, temperature);
	      
}


void castelLinkData::convertValues(void) 
{
  if (raw == nullptr)
    chSysHalt ("convertValues : raw  == nullptr");

  
  const float cal_coeff_0 = std::min(raw->get_temp_linear_or_cal(), raw->get_temp_ntc_or_cal());
  const float cal_coeff_1 = raw->get_calibration_1ms();
  const bool temp_NTC = raw->get_temp_linear_or_cal() < raw->get_temp_ntc_or_cal();

  for (size_t i=0; i< datas.size(); i++) {
    if ((i+1 < raw->get_raw_len()) && (i < scale_coeffs.size()))
      datas[i] = ((raw->get_raw_ref()[i+1] - cal_coeff_0) / cal_coeff_1) * scale_coeffs[i];
  }

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
  simpleMsgSend(CASTELLINK::STREAM_TELEMETRY_PTR, reinterpret_cast<uint8_t *> (this), sizeof(*this));
}




/*
#                                _    _    _                      _            
#                               | |  | |  | |                    | |           
#                  ___    __ _  | |  | |  | |__     __ _    ___  | | _         
#                 / __|  / _` | | |  | |  | '_ \   / _` |  / __| | |/ /        
#                | (__  | (_| | | |  | |  | |_) | | (_| | | (__  |   <         
#                 \___|  \__,_| |_|  |_|  |_.__/   \__,_|  \___| |_|\_\        
*/

//
//  PWM
// 
static void pwmStartIcu_cb (PWMDriver *pwmd)
{
  (void) pwmd;
  chSysLockFromISR();
  icuStartCaptureI(&CASTELLINK::ICU);
  icuEnableNotificationsI(&CASTELLINK::ICU);

  chSysUnlockFromISR();
}


static void pwmModeHighZ_cb (PWMDriver *pwmd)
{
  (void) pwmd;
  chSysLockFromISR();
  //pwmMaskChannelOutput(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL, true);
  pwmMaskChannelSide(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL, PWM_NORMAL, true);
  //  palSetLine(LINE_DBG_HiZ);
  chSysUnlockFromISR();
}

static void pwmModePushpull_cb (PWMDriver *pwmd)
{
  (void) pwmd;
  chSysLockFromISR();
  
  //pwmMaskChannelOutput(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL, false);
  pwmMaskChannelSide(&CASTELLINK::PWM, CASTELLINK::PWM_COMMAND_CHANNEL, PWM_NORMAL, false);
  //  palClearLine(LINE_DBG_HiZ);
  icuStopCaptureI(&CASTELLINK::ICU);
  if (icuOverflow && currentRaw) {
    icuOverflow = false;
    currentRaw->resetIndex();
  }
  
  
  chSysUnlockFromISR();
}




  
//
//  ICU
// 
static void icuWidth_cb (ICUDriver *icup)
{
  chSysLockFromISR();
  const icucnt_t width = icuGetWidthX(icup);

  if (currentRaw) {
    if (width >= CASTELLINK::ICU_MINPULSE_TICK && width <= CASTELLINK::ICU_MAXPULSE_TICK) {
      icuStopCaptureI(&CASTELLINK::ICU);
      if (currentRaw->push(width)) {
	chFifoSendObjectI(&raw_fifo, currentRaw);
	currentRaw =  static_cast<castelLinkRawData *> (chFifoTakeObjectI(&raw_fifo));
      }
    }
  } else { // currentRaw == nullptr
    currentRaw = static_cast<castelLinkRawData *> (chFifoTakeObjectI(&raw_fifo));
  }

  chSysUnlockFromISR();
}


static void icuTimout_cb (ICUDriver *icud)
{
  (void) icud;

  icuOverflow = true;
  palSetLine(LINE_DBG_HiZ);
  chSysPolledDelayX(10);
  palClearLine(LINE_DBG_HiZ);
}

static void telemetryReceive_cb(const uint8_t *buffer, const size_t len,  void * const userData)
{
  (void) userData;
  
  if (len != sizeof(TelemetryDownMsg)) {
    DebugTrace ("Msg len error : rec %u instead of waited %u", len, sizeof(TelemetryDownMsg));
  } else {
    const TelemetryDownMsg *msg = reinterpret_cast<const TelemetryDownMsg *> (buffer);
    switch (msg->msgId) {
    case PWM_ORDER : castelLinkSetDuty(msg->value[0]); break;
    case CALIBRATE : DebugTrace ("Calibrate not yet implemented"); break;
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
    castelLinkData processedData(rawData, 0);
    processedData.sendTelemetry();
    //    rawData->dbgTrace();
     processedData.dbgTrace();
    chFifoReturnObject(&raw_fifo, rawData);
  }
}


/*
#                     _    _        __ _         
#                    | |  | |      / _` |        
#                  __| |  | |__   | (_| |        
#                 / _` |  | '_ \   \__, |        
#                | (_| |  | |_) |   __/ |        
#                 \__,_|  |_.__/   |___/         
#                               _     __         
#                              | |   / _|        
#                 ___     ___  | |  | |_         
#                / __|   / _ \ | |  |  _|        
#                \__ \  |  __/ | |  | |          
#                |___/   \___| |_|  |_|          
#                 _                    _            
#                | |                  | |           
#                | |_     ___   ___   | |_          
#                | __|   / _ \ / __|  | __|         
#                \ |_   |  __/ \__ \  \ |_          
#                 \__|   \___| |___/   \__|         
*/
static void initPulse_cb(PWMDriver *pwmp)
{
  (void) pwmp;
  static constexpr uint32_t pulses[] = {
    0,    // no pulse
    1000, // calib
    1500, // voltage
    2000, // ripple
    2000, // current
    2000, // throttle
    2000, // power
    2000, // rpm
    2000, // bec volt
    5000, // bec current
    500,  // cal
    600  // temp
  };



  static uint32_t count =0;
  
  if (count == 11) {
    count = 0;
    return;
  }

  chSysLockFromISR();
  const uint32_t pulseDurationUs = pulses[count+1];

  gptStartOneShotI(&GPTD6, pulseDurationUs);
  gptStartOneShotI(&GPTD7, pulseDurationUs+10);
  count++;
  chSysUnlockFromISR();
}

static void gpt6_cb(GPTDriver *gptp)
{
  (void) gptp;
    palClearLine(LINE_TEST_PULSE);
}

static void gpt7_cb(GPTDriver *gptp)
{
  (void) gptp;
    palSetLine(LINE_TEST_PULSE);
}
