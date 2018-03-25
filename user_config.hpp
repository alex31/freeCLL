#pragma once


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

  // overflow after 8.2ms (frame start detection)
  static inline constexpr uint32_t	ICU_TIMFREQ      = 8_mhz; 
  static inline constexpr uint32_t	ICU_MINPULSE_US  = 400;
  static inline constexpr uint32_t	ICU_MAXPULSE_US  = 6000;

  // to put telemetry on serial over usb, disable TRACE and choose SD2
  static inline constexpr SerialDriver&	SD_TELEMETRY    = SD2;
  static inline constexpr uint32_t	TELEMETRY_BAUD  = 115200U;

  // in absence uplink message on serial link, engine is stop
  // disable test if 0
  static inline constexpr uint32_t	SHUTDOWN_WITHOUT_TELEMETRY_MS  = 5000U;

  static inline constexpr size_t	FIFO_SIZE    = 4;
  
};

