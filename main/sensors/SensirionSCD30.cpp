#include "SensirionSCD30.h"

#include <cmath>
#include <stdint.h>
#include "drivers/sensirion/scd30_i2c.h"
#include "drivers/sensirion/sensirion_common.h"
#include "drivers/sensirion/sensirion_i2c_hal.h"

bool SensirionSCD30::Init()
{
  sensirion_i2c_hal_init();
  scd30_init(SCD30_I2C_ADDR_61);

  AirQualitySensor::Init();

  ActivateAutomaticSelfCalibration();
  StartContinuousMeasurement();

  return true;
}

std::set<Sensor::MeasurementType> SensirionSCD30::GetSupportedMeasurements() const
{
    // Return the measurement types supported by SEN66
    return {
        Sensor::MeasurementType::CO2,
        Sensor::MeasurementType::RelativeHumidity,
        Sensor::MeasurementType::Temperature
    };
}

std::string SensirionSCD30::GetProductName()
{
  return "SCD30";
}

std::string SensirionSCD30::GetVendorName()
{
  return "Sensirion";
}

int SensirionSCD30::GetFirmwareVersion(int* firmwareMajorVersion, int* firmwareMinorVersion)
{
  uint8_t majorVersion;
  uint8_t minorVersion;

  int16_t status = scd30_read_firmware_version(&majorVersion, &minorVersion);

  *firmwareMajorVersion = majorVersion;
  *firmwareMinorVersion = minorVersion;

  return status;
}

std::vector<AirQualitySensor::Measurement> SensirionSCD30::ReadAllMeasurements()
{
  float co2_concentration;
  float temperature;
  float humidity;

  std::vector<AirQualitySensor::Measurement> measurements;

  int16_t local_error = 0;
  local_error = scd30_await_data_ready();
  if (local_error != NO_ERROR) {
      return measurements; // Return an empty vector on error
  }

  local_error =
      scd30_read_measurement_data(&co2_concentration, &temperature, &humidity);
  if (local_error != NO_ERROR) {
      return measurements; // Return an empty vector on error
  }

  measurements.push_back({MeasurementType::RelativeHumidity, humidity});
  measurements.push_back({MeasurementType::Temperature, temperature});
  measurements.push_back({MeasurementType::CO2, static_cast<float>(co2_concentration)});

  return measurements;
}

// Methods from TemperatureSensor
std::optional<float> SensirionSCD30::MeasureTemperature()
{
    float co2_concentration;
    float temperature;
    float humidity;

    int16_t local_error = 0;
    local_error = scd30_await_data_ready();
    if (local_error != NO_ERROR) {
        return std::nullopt; // Return nullopt on error
    }

    local_error =
        scd30_read_measurement_data(&co2_concentration, &temperature, &humidity);
    if (local_error != NO_ERROR) {
        return std::nullopt; // Return nullopt on error
    }

    return temperature;
}

// Methods from RelativeHumiditySensor
std::optional<float> SensirionSCD30::MeasureRelativeHumidity()
{
    float co2_concentration;
    float temperature;
    float humidity;

    int16_t local_error = 0;
    local_error = scd30_await_data_ready();
    if (local_error != NO_ERROR) {
        return std::nullopt; // Return nullopt on error
    }

    local_error =
        scd30_read_measurement_data(&co2_concentration, &temperature, &humidity);
    if (local_error != NO_ERROR) {
        return std::nullopt; // Return nullopt on error
    }

    return humidity;
}

int SensirionSCD30::ActivateAutomaticSelfCalibration()
{
  int16_t status = scd30_activate_auto_calibration(1);
  return status;
}

int SensirionSCD30::SetAltitude(float altitude)
{
  int16_t status = scd30_set_altitude_compensation(altitude);
  return status;
}

int SensirionSCD30::SetAmbientPressure(float ambientPressureKiloPascal)
{
  // Round the float to the nearest integer and convert to uint16_t
  uint16_t pressureHektoPascal = static_cast<uint16_t>(std::round(ambientPressureKiloPascal * 10.0f));

  int16_t status = scd30_start_periodic_measurement(pressureHektoPascal);

  return status;
}

int SensirionSCD30::StartContinuousMeasurement()
{
  uint16_t ambient_pressure = 0; // Disable pressure compensation
  int16_t status = scd30_start_periodic_measurement(ambient_pressure);

  return status;
}
