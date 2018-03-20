#pragma once

#include <ch.h>
#include <hal.h>


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

namespace CASTELLINK {
  // USER CONSTANTS
  static inline constexpr PWMDriver&	PWM      = PWMD2;
  static inline constexpr uint32_t	PWM_COMMAND_CHANNEL = 1_timChannel;
  static inline constexpr uint32_t	PWM_HIGHZ_CHANNEL = 2_timChannel;
  static inline constexpr uint32_t	PWM_PUSHPULL_CHANNEL = 3_timChannel;
  static inline constexpr uint32_t	PWM_FREQ = 50_hz;
  static inline constexpr uint32_t	TICK_PER_PERIOD = 20000;
  static inline constexpr uint32_t	HIGHZ_TIMESHIFT_MICROSECONDS = 100;
  static inline constexpr uint32_t	PUSHPULL_DUTY_MILLISECONDS = 14;

  static inline constexpr ICUDriver&	ICU		 = ICUD1;
  static inline constexpr icuchannel_t	ICU_CHANNEL      = ICU_CHANNEL_1;
  static inline constexpr uint32_t	ICU_TIMFREQ      = 8_mhz; // overflow after 8ms (frame start detection)
  static inline constexpr uint32_t	ICU_MINPULSE_US  = 400;
  static inline constexpr uint32_t	ICU_MAXPULSE_US  = 6000;


  static inline constexpr SerialDriver&	SD_TELEMETRY    = SD1;
  static inline constexpr uint32_t	TELEMETRY_BAUD  = 115200U;

  static inline constexpr size_t	FIFO_SIZE    = 4;
  

  // COMPUTED CONSTANTS, don't EDIT THIS SECTION until you know what you do
  
  static inline constexpr uint32_t	TICK_FREQ = PWM_FREQ * TICK_PER_PERIOD;
  static inline constexpr uint32_t	HIGHZ_TIMESHIFT_TICKS = usec2rtc(TICK_FREQ, HIGHZ_TIMESHIFT_MICROSECONDS);
  static inline constexpr uint32_t	PUSHPULL_DUTY_TICKS = msec2rtc(TICK_FREQ, PUSHPULL_DUTY_MILLISECONDS);
  static inline constexpr uint32_t	ICU_MINPULSE_TICK      = usec2rtc(ICU_TIMFREQ, ICU_MINPULSE_US);
  static inline constexpr uint32_t	ICU_MAXPULSE_TICK      = usec2rtc(ICU_TIMFREQ, ICU_MAXPULSE_US);
  static constexpr BaseSequentialStream* STREAM_TELEMETRY_PTR = (BaseSequentialStream *) &SD_TELEMETRY;
};

#ifdef TRACE
 static inline constexpr bool	USE_SHELL    = true;
 static inline constexpr SerialDriver&	SD_SHELL        = SD2;  // wired on NUCLEO32
 static constexpr BaseSequentialStream* STREAM_SHELL_PTR = (BaseSequentialStream *) &SD_SHELL;
#else
 static inline constexpr bool	USE_SHELL    = false;
#endif
