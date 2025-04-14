#include "SensirionSEN66.h"
#include <stdint.h>
#include "drivers/sen66_i2c.h"
#include "drivers/sensirion_common.h"
#include "drivers/sensirion_i2c_hal.h"

void SensirionSEN66::Init()
{
  sensirion_i2c_hal_init();
  sen66_init(SEN66_I2C_ADDR_6B);
}

int SensirionSEN66::ReadMeasuredValues(MeasuredValues* measuredValues)
{
  uint16_t particulateMatter1p0 = 0;
  uint16_t particulateMatter2p5 = 0;
  uint16_t particulateMatter4p0 = 0;
  uint16_t particulateMatter10p0 = 0;
  int16_t ambientHumidity = 0;
  int16_t ambientTemperature = 0;
  int16_t vocIndex = 0;
  int16_t noxIndex = 0;
  uint16_t co2 = 0;

  int16_t status = sen66_read_measured_values_as_integers(
    &particulateMatter1p0,
    &particulateMatter2p5,
    &particulateMatter4p0,
    &particulateMatter10p0,
    &ambientHumidity,
    &ambientTemperature,
    &vocIndex,
    &noxIndex,
    &co2);

    if (status != NO_ERROR) {
      return status;
    }

    measuredValues->ParticulateMatter1p0 = particulateMatter1p0 / 10.0f;
    measuredValues->ParticulateMatter2p5 = particulateMatter2p5 / 10.0f;
    measuredValues->ParticulateMatter4p0 = particulateMatter4p0 / 10.0f;
    measuredValues->ParticulateMatter10p0 = particulateMatter10p0 / 10.0f;
    measuredValues->AmbientHumidity = ambientHumidity / 100.0f;
    measuredValues->AmbientTemperature = ambientTemperature / 200.0f;
    measuredValues->VOCIndex = vocIndex / 10.0f;
    measuredValues->NOxIndex = noxIndex / 10.0f;
    measuredValues->CO2 = co2;

    return status;
}

int SensirionSEN66::SetSensorAltitude(float altitude)
{
  int16_t status = sen66_set_sensor_altitude(altitude);
  return status;
}

int SensirionSEN66::StartContiniousMeasurement()
{
  int16_t status = sen66_start_continuous_measurement();

  return status;
}