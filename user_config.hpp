#pragma once


namespace CASTELLINK {
  // USER CONSTANTS
  static inline constexpr PWMDriver&	PWM      = PWMD1;
  static inline constexpr uint32_t	PWM_COMMAND_CH_1 = 1_timChannel;
  static inline constexpr uint32_t	PWM_HIGHZ_CH_1 = 2_timChannel;
  static inline constexpr uint32_t	PWM_HIGHZ_CH_2 = 3_timChannel;
  static inline constexpr uint32_t	PWM_COMMAND_CH_2 = 4_timChannel;
  static inline constexpr uint32_t	PWM_FREQ = 50_hz;
  static inline constexpr uint32_t	TICK_PER_PERIOD = 20000;
  static inline constexpr uint32_t	HIGHZ_TIMESHIFT_MICROSECONDS = 100;
  static inline constexpr uint32_t	PUSHPULL_DUTY_MILLISECONDS = 14;

  static inline constexpr ICUDriver&	ICU1		 = ICUD2;
  static inline constexpr ICUDriver&	ICU2		 = ICUD15;
  static inline constexpr icuchannel_t	ICU1_CHANNEL      = ICU_CHANNEL_1;
  static inline constexpr icuchannel_t	ICU2_CHANNEL      = ICU_CHANNEL_2;

  static inline constexpr GPTDriver&	GPT_PUSHPULL	  = GPTD6;
  static inline constexpr uint32_t	GPT_PP_FREQ       = 1_mhz;

  // overflow after 6.5ms (frame start detection)
  static inline constexpr uint32_t	ICU_TIMFREQ      = 10_mhz; 
  static inline constexpr uint32_t	ICU_MINPULSE_US  = 400;
  static inline constexpr uint32_t	ICU_MAXPULSE_US  = 6000;

  // to put telemetry on serial over usb, disable TRACE and choose SD2
  static inline constexpr SerialDriver&	SD_TELEMETRY    = SD2;
  static inline constexpr uint32_t	TELEMETRY_BAUD  = 115200U;

  // in absence uplink message on serial link, engine is stop
  // disable test if 0
  static inline constexpr uint32_t	SHUTDOWN_WITHOUT_TELEMETRY_MS  = 5000U;

  static inline constexpr size_t	FIFO_SIZE    = 8;
  
};

