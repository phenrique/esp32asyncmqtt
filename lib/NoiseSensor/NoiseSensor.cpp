#include "NoiseSensor.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>

esp_adc_cal_characteristics_t adc_cal;
const int numReadings = 50;
int readings[numReadings];
int readIndex = 0;
long total = 0;
int mean = 0;

void smooth(TimerHandle_t xTime);

NoiseSensor::NoiseSensor()
{
  adc1_config_width(ADC_WIDTH_BIT_12);

  adc1_config_channel_atten(GPIO34, ADC_ATTEN_DB_12); // ADC_ATTEN_DB_12 mesmo que ADC_ATTEN_DB_11
}

void NoiseSensor::begin()
{
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_cal); // Inicializa a estrutura de calibracao
}

void NoiseSensor::startSmoothing()
{
  TimerHandle_t sensorTimer = xTimerCreate("SensorTimer", pdMS_TO_TICKS(1), pdTRUE, NULL, smooth);
  xTimerStart(sensorTimer, 0);
}

u_int32_t readVoltage()
{
  return esp_adc_cal_raw_to_voltage(adc1_get_raw(GPIO34), &adc_cal);
}

u_int32_t NoiseSensor::read()
{
  return readVoltage();
}

u_int32_t NoiseSensor::readSmoothed()
{
  return mean;
}

void smooth(TimerHandle_t xTime)
{
  static int *currentReading = readings;
  total -= *currentReading;
  *currentReading = readVoltage();
  total += *currentReading;
  currentReading++;
  if (currentReading >= readings + numReadings)
    currentReading = readings;
  mean = total / numReadings;
}
