#pragma once

#include <ch.h>
#include <hal.h>

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

// if USE_SHELL == false, should not be compiled with -DTRACE (see Makefile)
static inline constexpr bool	USE_SHELL    = true;


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
  static inline constexpr SerialDriver&	SD_SHELL        = SD2;  // wired on NUCLEO32
  static inline constexpr uint32_t	TELEMETRY_BAUD  = 115200U;

  static inline constexpr size_t	FIFO_SIZE    = 4;
  

  // COMPUTED CONSTANTS, don't EDIT THIS SECTION until you know what you do
  static inline constexpr uint32_t	TICK_FREQ = PWM_FREQ * TICK_PER_PERIOD;
  static inline constexpr uint32_t	HIGHZ_TIMESHIFT_TICKS = (HIGHZ_TIMESHIFT_MICROSECONDS *
								 TICK_FREQ) / 1e6;
  static inline constexpr uint32_t	PUSHPULL_DUTY_TICKS = (PUSHPULL_DUTY_MILLISECONDS *
							       TICK_FREQ) / 1e3;

  static inline constexpr uint32_t	ICU_MINPULSE_TICK      = (static_cast<uint64_t> (ICU_TIMFREQ) *
								  static_cast<uint64_t> (ICU_MINPULSE_US))
								  / static_cast<uint64_t> (1e6);
  static inline constexpr uint32_t	ICU_MAXPULSE_TICK      = (static_cast<uint64_t> (ICU_TIMFREQ) *
								  static_cast<uint64_t> (ICU_MAXPULSE_US))
    								  / static_cast<uint64_t> (1e6);


  static constexpr BaseSequentialStream* STREAM_TELEMETRY_PTR = (BaseSequentialStream *) &SD_TELEMETRY;
};
