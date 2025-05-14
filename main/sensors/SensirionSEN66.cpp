#include "SensirionSEN66.h"

#include <cmath>
#include <stdint.h>
#include "drivers/sensirion/sen66_i2c.h"
#include "drivers/sensirion/sensirion_common.h"
#include "drivers/sensirion/sensirion_i2c_hal.h"

namespace {
    constexpr uint16_t INVALID_UINT16 = 0xFFFF;
    constexpr int16_t INVALID_INT16 = 0x7FFF;
    constexpr float PM_CONVERSION_FACTOR = 10.0f;
    constexpr float HUMIDITY_CONVERSION_FACTOR = 100.0f;
    constexpr float TEMPERATURE_CONVERSION_FACTOR = 200.0f;
    constexpr float VOC_CONVERSION_FACTOR = 10.0f;
    constexpr float NOX_CONVERSION_FACTOR = 10.0f;
}

// Initialize SensorData with default invalid values
SensirionSEN66::SensorData::SensorData()
    : pm1p0(INVALID_UINT16), pm2p5(INVALID_UINT16), pm4p0(INVALID_UINT16), pm10p0(INVALID_UINT16),
      humidity(INVALID_INT16), temperature(INVALID_INT16), vocIndex(INVALID_INT16), noxIndex(INVALID_INT16),
      co2(INVALID_UINT16) {
}

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

// Private helper function to read all sensor values
int16_t SensirionSEN66::ReadSensorData(SensorData& data) {
    return sen66_read_measured_values_as_integers(
        &data.pm1p0, &data.pm2p5, &data.pm4p0, &data.pm10p0,
        &data.humidity, &data.temperature, &data.vocIndex, &data.noxIndex, &data.co2);
}

std::vector<AirQualitySensor::Measurement> SensirionSEN66::ReadAllMeasurements() {
      SensorData data;
      std::vector<AirQualitySensor::Measurement> measurements;

      if (ReadSensorData(data) != NO_ERROR) {
          return measurements; // Return empty vector on error
      }

      // Add valid measurements to the vector
      if (data.pm1p0 != INVALID_UINT16) {
          measurements.push_back({MeasurementType::PM1p0, data.pm1p0 / PM_CONVERSION_FACTOR});
      }
      if (data.pm2p5 != INVALID_UINT16) {
          measurements.push_back({MeasurementType::PM2p5, data.pm2p5 / PM_CONVERSION_FACTOR});
      }
      if (data.pm4p0 != INVALID_UINT16) {
          measurements.push_back({MeasurementType::PM4p0, data.pm4p0 / PM_CONVERSION_FACTOR});
      }
      if (data.pm10p0 != INVALID_UINT16) {
          measurements.push_back({MeasurementType::PM10p0, data.pm10p0 / PM_CONVERSION_FACTOR});
      }
      if (data.humidity != INVALID_INT16) {
          measurements.push_back({MeasurementType::RelativeHumidity, data.humidity / HUMIDITY_CONVERSION_FACTOR});
      }
      if (data.temperature != INVALID_INT16) {
          measurements.push_back({MeasurementType::Temperature, data.temperature / TEMPERATURE_CONVERSION_FACTOR});
      }
      if (data.vocIndex != INVALID_INT16) {
          measurements.push_back({MeasurementType::VOC, data.vocIndex / VOC_CONVERSION_FACTOR});
      }
      if (data.noxIndex != INVALID_INT16) {
          measurements.push_back({MeasurementType::NOx, data.noxIndex / NOX_CONVERSION_FACTOR});
      }
      if (data.co2 != INVALID_UINT16) {
          measurements.push_back({MeasurementType::CO2, static_cast<float>(data.co2)});
      }

      return measurements;
}

std::optional<float> SensirionSEN66::MeasureTemperature() {
    SensorData data;
    if (ReadSensorData(data) != NO_ERROR || data.temperature == INVALID_INT16) {
        return std::nullopt;
    }
    return data.temperature / TEMPERATURE_CONVERSION_FACTOR;
}

std::optional<float> SensirionSEN66::MeasureRelativeHumidity() {
    SensorData data;
    if (ReadSensorData(data) != NO_ERROR || data.humidity == INVALID_INT16) {
        return std::nullopt;
    }
    return data.humidity / HUMIDITY_CONVERSION_FACTOR;
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
