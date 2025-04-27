#include "SensirionSEN66.h"
#include <stdint.h>
#include "drivers/sen66_i2c.h"
#include "drivers/sensirion_common.h"
#include "drivers/sensirion_i2c_hal.h"

void SensirionSEN66::Init()
{
  sensirion_i2c_hal_init();
  sen66_init(SEN66_I2C_ADDR_6B);

  AirQualitySensor::Init();
}

std::set<AirQualitySensor::MeasurementType> SensirionSEN66::GetSupportedMeasurements() const
{
    // Return the measurement types supported by SEN66
    return {
        AirQualitySensor::MeasurementType::ParticulateMatter1p0,
        AirQualitySensor::MeasurementType::ParticulateMatter2p5,
        AirQualitySensor::MeasurementType::ParticulateMatter4p0,
        AirQualitySensor::MeasurementType::ParticulateMatter10p0,
        AirQualitySensor::MeasurementType::AmbientHumidity,
        AirQualitySensor::MeasurementType::AmbientTemperature,
        AirQualitySensor::MeasurementType::VOCIndex,
        AirQualitySensor::MeasurementType::NOxIndex,
        AirQualitySensor::MeasurementType::CO2
    };
}

std::vector<AirQualitySensor::Measurement> SensirionSEN66::ReadAllMeasurements()
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

    std::vector<AirQualitySensor::Measurement> measurements;

    if (status != NO_ERROR) {
      return measurements; // Return an empty vector on error
    }

    if (particulateMatter1p0 != 0xFFFF) {
      measurements.push_back({MeasurementType::ParticulateMatter1p0, particulateMatter1p0 / 10.0f});
    }

    if (particulateMatter2p5 != 0xFFFF) {
      measurements.push_back({MeasurementType::ParticulateMatter2p5, particulateMatter2p5 / 10.0f});
    }

    if (particulateMatter4p0 != 0xFFFF) {
      measurements.push_back({MeasurementType::ParticulateMatter4p0, particulateMatter4p0 / 10.0f});
    }

    if (particulateMatter10p0 != 0xFFFF) {
      measurements.push_back({MeasurementType::ParticulateMatter10p0, particulateMatter10p0 / 10.0f});
    }

    if (ambientHumidity != 0x7FFF) {
      measurements.push_back({MeasurementType::AmbientHumidity, ambientHumidity / 100.0f});
    }

    if (ambientTemperature != 0x7FFF) {
      measurements.push_back({MeasurementType::AmbientTemperature, ambientTemperature / 200.0f});
    }

    if (vocIndex != 0x7FFF) {
      measurements.push_back({MeasurementType::VOCIndex, vocIndex / 10.0f});
    }

    if (noxIndex != 0x7FFF) {
      measurements.push_back({MeasurementType::NOxIndex, noxIndex / 10.0f});
    }

    if (co2 != 0xFFFF) {
      measurements.push_back({MeasurementType::CO2, static_cast<float>(co2)});
    }

    return measurements;
}

int SensirionSEN66::SetSensorAltitude(float altitude)
{
  int16_t status = sen66_set_sensor_altitude(altitude);
  return status;
}

int SensirionSEN66::StartContinuousMeasurement()
{
  int16_t status = sen66_start_continuous_measurement();

  return status;
}