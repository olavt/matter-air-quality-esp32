#pragma once

#include "Sensor.h"
#include "TemperatureSensor.h"
#include "RelativeHumiditySensor.h"
#include <string>


class AirQualitySensor : public TemperatureSensor, public RelativeHumiditySensor
{

public:

    // Constructor
    explicit AirQualitySensor(float sensorAltitude = 0.0f)
        : m_sensorAltitude(sensorAltitude)
    {
    }

    // Virtual destructor for proper cleanup
    virtual ~AirQualitySensor() = default;

    // Initialize the sensor
    bool Init() override;
    
    // Methods from Sensor
    std::vector<Measurement> ReadAllMeasurements() override = 0;

    // Methods from TemperatureSensor
    std::optional<float> MeasureTemperature() override = 0;

    // Methods from RelativeHumiditySensor
    std::optional<float> MeasureRelativeHumidity() override = 0;

    virtual std::string GetProductName() = 0;

    virtual std::string GetVendorName() = 0;

    virtual int GetFirmwareVersion(int* firmwareMajorVersion, int* firmwareMinorVersion) = 0;

    // Get the set of measurement types supported by this sensor
    virtual std::set<MeasurementType> GetSupportedMeasurements() const = 0;

    virtual int ActivateAutomaticSelfCalibration() = 0;

    virtual int SetAmbientPressure(float ambientPressureKiloPascal) = 0;

protected:

    virtual int SetAltitude(float altitude) = 0;

    virtual int StartContinuousMeasurement() = 0;

    float m_sensorAltitude;
};
