#pragma once
#include <stdint.h>
#include <stddef.h>
#include <array>


namespace CASTELLINK {
  static inline constexpr int16_t	PWM_DISABLE = -1;
}

class castelLinkRawData
{
public:
  castelLinkRawData();
  void  resetIndex(void);
  bool  push(const uint16_t val);
  void dbgTrace(void) const ;
private:
  static constexpr size_t raw_len = 11;
  volatile size_t  index;
  union {
    struct {
      uint16_t	calibration_1ms;
      uint16_t	bat_voltage;
      uint16_t	ripple_voltage;
      uint16_t	current;
      uint16_t	throttle;
      uint16_t	power;
      uint16_t	rpm;
      uint16_t	bec_voltage;
      uint16_t	bec_current;
      uint16_t	temp_linear_or_cal;
      uint16_t	temp_ntc_or_cal;
    };
    std::array <uint16_t, raw_len> raw;
  };
  uint8_t channel;

public:
  uint16_t	   get_calibration_1ms (void) const { return calibration_1ms; };
  uint16_t	   get_temp_linear_or_cal (void) const { return temp_linear_or_cal; };
  uint16_t	   get_temp_ntc_or_cal (void) const { return temp_ntc_or_cal; };
  const  std::array <uint16_t, raw_len> &
		   get_raw_ref	    (void) const { return raw; };
  size_t           get_raw_len	    (void) const { return raw_len;};
  void     setChannel(const  uint8_t _channel)  {channel = _channel;};
  uint8_t  getChannel(void)  const {return channel;};
};

typedef enum  : uint16_t {PWM_ORDER=0, CALIBRATE} MessageId;
typedef struct {
  MessageId msgId;
  int16_t  linkId;
  int16_t  duty;
} TelemetryDownMsg;

class castelLinkData
{
  static constexpr std::array<float, 10> scale_coeffs {
      20,
      4,
      50,
      1,
      0.2502,
      20416.7,
      4,
      4,
      30,
      63.8125};

  
public:
  castelLinkData();
  castelLinkData(const castelLinkRawData* _raw);
  void  sendTelemetry(void);
  void populate(const castelLinkRawData* _raw);
  void dbgTrace(void) const ;
  
private:
  void convertValues(void);
  
private:

  struct {
    union {
      struct {
	float	bat_voltage;
	float	ripple_voltage;
	float	current;
	float	throttle;
	float	power;
	float	rpm;
	float	bec_voltage;
	float	bec_current;
	float	temperature;             
      };
      std::array<float, 9> datas;
    };
    uint32_t	channel;
  };
  const castelLinkRawData* raw;
};  

  
void castelLinkStart(void);
void castelLinkSetDuty(const uint8_t linkIdx, const int16_t dutyPerTenThousand);
