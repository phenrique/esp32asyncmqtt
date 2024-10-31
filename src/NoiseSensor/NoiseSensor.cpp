#include "NoiseSensor.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

esp_adc_cal_characteristics_t adc_cal;

NoiseSensor::NoiseSensor()
{
  adc1_config_width(ADC_WIDTH_BIT_12);

  // full voltage range
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

}

void NoiseSensor::begin(){
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_cal); // Inicializa a estrutura de calibracao
}

u_int32_t NoiseSensor::read()
{
  uint32_t voltage = adc1_get_raw(ADC1_CHANNEL_6);      // Converte e calibra o valor lido (RAW) para mV
  return esp_adc_cal_raw_to_voltage(voltage, &adc_cal); // Converte e calibra o valor lido (RAW) para mV
}
