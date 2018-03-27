#pragma once

#include <ch.h>
#include <hal.h>
#include "hal_stm32_lld_icu_opt.h"
#include <utility>

/*
 to enable self test pulse generation :
	search SELFTEST_PULSES_ENABLED in mcuconf.h

 to enable  chibios internal tests and debug assert :
	search DEBUG_ASSERTS_ENABLED in mcuconf.h

 to enable optimised compilation : Makefile

 to enable TRACE and shell : -DTRACE in Makefile

 

*/

#undef DebugTrace
#if defined TRACE 
#include "printf.h"
#define DebugTrace(fmt, ...) chprintf (STREAM_SHELL_PTR, fmt "\r\n", ## __VA_ARGS__ )
#else
#define DebugTrace(...) 
#endif // TRACE



static inline constexpr uint32_t operator"" _timChannel (unsigned long long int channel)
{
  return channel - 1U;
}
static inline constexpr uint32_t operator"" _hz (unsigned long long int freq)
{
  return freq;
}
static inline constexpr uint32_t operator"" _khz (unsigned long long int freq)
{
  return freq * 1000UL;
}
static inline constexpr uint32_t operator"" _mhz (unsigned long long int freq)
{
  return freq * 1000_khz;
}

static inline constexpr uint32_t usec2rtc ( const uint32_t frequency, const uint32_t usec)
{
  return (static_cast<uint64_t> (frequency) *
	  static_cast<uint64_t> (usec)) / static_cast<uint64_t> (1e6);
}

static inline constexpr uint32_t msec2rtc ( const uint32_t frequency, const uint32_t usec)
{
  return (static_cast<uint64_t> (frequency) *
	  static_cast<uint64_t> (usec)) / static_cast<uint64_t> (1e3);
}


struct PwmCoupledIcuParam {
  const PWMDriver& pwmr;
  const ICUDriver& icur;
  const pwmchannel_t channel;
};



#include "user_config.hpp"

namespace CASTELLINK {
  // COMPUTED CONSTANTS, don't EDIT THIS SECTION until you know what you do
  static inline constexpr uint32_t	TICK_FREQ = PWM_FREQ * TICK_PER_PERIOD;
  static inline constexpr uint32_t	HIGHZ_TIMESHIFT_TICKS = usec2rtc(TICK_FREQ, HIGHZ_TIMESHIFT_MICROSECONDS);
  static inline constexpr uint32_t	PUSHPULL_TIMESHIFT_TICKS = msec2rtc(GPT_PP_FREQ, PUSHPULL_DUTY_MILLISECONDS);
  static inline constexpr uint32_t	ICU_MINPULSE_TICK      = usec2rtc(ICU_TIMFREQ, ICU_MINPULSE_US);
  static inline constexpr uint32_t	ICU_MAXPULSE_TICK      = usec2rtc(ICU_TIMFREQ, ICU_MAXPULSE_US);
  static constexpr BaseSequentialStream* STREAM_TELEMETRY_PTR = (BaseSequentialStream *) &SD_TELEMETRY;
};



#ifdef TRACE
 static inline constexpr bool	USE_SHELL    = true;
 static inline constexpr SerialDriver&	SD_SHELL        = SD2;  // noy in userConfig since wired on NUCLEO32
 static constexpr BaseSequentialStream* STREAM_SHELL_PTR = (BaseSequentialStream *) &SD_SHELL;
#else
 static inline constexpr bool	USE_SHELL    = false;
#endif



// using Item = std::pair<ICUDriver*, int>;
// constexpr Item map_items[] = {
//     { &ICUD1, 1 },
//     { &ICUD15, 2 },
// };
