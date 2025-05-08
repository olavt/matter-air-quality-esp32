#include "SensirionSEN66.h"

#include <cmath>
#include <stdint.h>
#include "drivers/sensirion/sen66_i2c.h"
#include "drivers/sensirion/sensirion_common.h"
#include "drivers/sensirion/sensirion_i2c_hal.h"

bool SensirionSEN66::Init()
{
  sensirion_i2c_hal_init();
  sen66_init(SEN66_I2C_ADDR_6B);

  AirQualitySensor::Init();

  ActivateAutomaticSelfCalibration();
  StartContinuousMeasurement();

  return true;
}

std::set<AirQualitySensor::MeasurementType> SensirionSEN66::GetSupportedMeasurements() const
{
    // Return the measurement types supported by SEN66
    return {
        Sensor::MeasurementType::CO2,
        Sensor::MeasurementType::PM1p0,
        Sensor::MeasurementType::PM2p5,
        Sensor::MeasurementType::PM4p0,
        Sensor::MeasurementType::PM10p0,
        Sensor::MeasurementType::RelativeHumidity,
        Sensor::MeasurementType::Temperature,
        Sensor::MeasurementType::VOC,
        Sensor::MeasurementType::NOx
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
      measurements.push_back({MeasurementType::PM1p0, particulateMatter1p0 / 10.0f});
    }

    if (particulateMatter2p5 != 0xFFFF) {
      measurements.push_back({MeasurementType::PM2p5, particulateMatter2p5 / 10.0f});
    }

    if (particulateMatter4p0 != 0xFFFF) {
      measurements.push_back({MeasurementType::PM4p0, particulateMatter4p0 / 10.0f});
    }

    if (particulateMatter10p0 != 0xFFFF) {
      measurements.push_back({MeasurementType::PM10p0, particulateMatter10p0 / 10.0f});
    }

    if (ambientHumidity != 0x7FFF) {
      measurements.push_back({MeasurementType::RelativeHumidity, ambientHumidity / 100.0f});
    }

    if (ambientTemperature != 0x7FFF) {
      measurements.push_back({MeasurementType::Temperature, ambientTemperature / 200.0f});
    }

    if (vocIndex != 0x7FFF) {
      measurements.push_back({MeasurementType::VOC, vocIndex / 10.0f});
    }

    if (noxIndex != 0x7FFF) {
      measurements.push_back({MeasurementType::NOx, noxIndex / 10.0f});
    }

    if (co2 != 0xFFFF) {
      measurements.push_back({MeasurementType::CO2, static_cast<float>(co2)});
    }

    return measurements;
}

int SensirionSEN66::ActivateAutomaticSelfCalibration()
{
  int16_t status = sen66_set_co2_sensor_automatic_self_calibration(1);
  return status;
}

std::string SensirionSEN66::GetProductName()
{
  return "SEN66";
}

std::string SensirionSEN66::GetVendorName()
{
  return "Sensirion";
}

int SensirionSEN66::GetFirmwareVersion(int* firmwareMajorVersion, int* firmwareMinorVersion)
{
  uint8_t majorVersion;
  uint8_t minorVersion;
  int16_t status = sen66_get_version(&majorVersion, &minorVersion);

  *firmwareMajorVersion = majorVersion;
  *firmwareMinorVersion = minorVersion;

  return status;
}

// Sets the current sensor altitude. The sensor altitude can be used for pressure compensation in the
// CO2 sensor. The default sensor altitude value is set to 0 meters above sea level. Valid input values
// are between 0 and 3000m.
// This configuration is volatile, i.e. the parameter will be reverted to its default value after a device reset.
int SensirionSEN66::SetAltitude(float altitude)
{
  int16_t status = sen66_set_sensor_altitude(altitude);
  return status;
}

// Sets the ambient pressure value. The ambient pressure can be used for pressure compensation in
// the CO2 sensor. Setting an ambient pressure overrides any pressure compensation based on a previously set
// sensor altitude. Use of this command is recommended for applications experiencing significant ambient
// pressure changes to ensure CO2 sensor accuracy. Valid input values are between 700 to 1’200 hPa. The default
// value is 1013 hPa.
// This configuration is volatile, i.e. the parameter will be reverted to its default value after a device reset.
int SensirionSEN66::SetAmbientPressure(float ambientPressureKiloPascal)
{
  // Round the float to the nearest integer and convert to uint16_t
  uint16_t pressureHektoPascal = static_cast<uint16_t>(std::round(ambientPressureKiloPascal * 10.0f));

  int16_t status = sen66_set_ambient_pressure(pressureHektoPascal);

  return status;
}

int SensirionSEN66::StartContinuousMeasurement()
{
  int16_t status = sen66_start_continuous_measurement();

  return status;
}
