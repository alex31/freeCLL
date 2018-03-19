#pragma once
#include <stdint.h>
#include <stddef.h>
#include <array>


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


public:
  uint16_t	   get_calibration_1ms (void) const { return calibration_1ms; };
  uint16_t	   get_temp_linear_or_cal (void) const { return temp_linear_or_cal; };
  uint16_t	   get_temp_ntc_or_cal (void) const { return temp_ntc_or_cal; };
  const  std::array <uint16_t, raw_len> &
		   get_raw_ref	    (void) const { return raw; };
  size_t           get_raw_len	    (void) const { return raw_len;};
};

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
  castelLinkData(const castelLinkRawData* _raw, const uint8_t _channel);
  void  sendTelemetry(void);
  void populate(const castelLinkRawData* _raw, const uint8_t _channel);
  void dbgTrace(void) const ;
  
private:
  void convertValues(void);
  
private:
  const castelLinkRawData* raw;

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
  uint8_t	channel;

};

  
void castelLinkStart(void);
void castelLinkSetDuty(uint16_t dutyPerTenThousand);
