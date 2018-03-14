#include "castel_link.hpp"

#include <ch.h>
#include <hal.h>
#include <algorithm>
#include <cmath>
#include "config.hpp"

castelLinkRawData::castelLinkRawData(void) : index{0}, raw{0} 
{
}

void castelLinkRawData::resetIndex(void)
{
  index = 0;
}


void  castelLinkRawData::push(const uint16_t val)
{
  raw[index++]=val;
}


castelLinkData::castelLinkData(const castelLinkRawData& _raw) : raw{_raw},
								bat_voltage{0},
								ripple_voltage{0},
								current{0},
								throttle{0},
								power{0},
								rpm{0},
								bec_voltage{0},
								bec_current{0},
								temperature{0}
	
{
  convertValues();
}

void castelLinkData::convertValues(void) 
{
  const float cal_coeff_0 = std::min(raw.get_temp_linear_or_cal(), raw.get_temp_ntc_or_cal());
  const float cal_coeff_1 = raw.get_calibration_1ms();
  const bool temp_NTC = raw.get_temp_linear_or_cal() < raw.get_temp_ntc_or_cal();

  for (size_t i=0; i< datas.size(); i++) {
    if ((i+1 < raw.get_raw_len()) && (i < scale_coeffs.size()))
      datas[i] = ((raw.get_raw_ref()[i+1] - cal_coeff_0) / cal_coeff_1) * scale_coeffs[i];
  }

  if (temp_NTC == true) {
    const float val = ((raw.get_temp_ntc_or_cal() - cal_coeff_0) / cal_coeff_1) * scale_coeffs[10];
    const float r0 = 10000;
    const float r2 = 10200;
    const float b = 3455;
    temperature = (1.0 / logf(val*r2/(255.0-val)/r0) / b + (1/298.0)) - 273;
  }
}

void castelLinkData::sendToHost(void) 
{
  
}

