#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <vector>

class AirQualitySensor
{
public:
    // Enum to define all possible measurement types
    enum class MeasurementType {
        ParticulateMatter1p0,
        ParticulateMatter2p5,
        ParticulateMatter4p0,
        ParticulateMatter10p0,
        AmbientHumidity,
        AmbientTemperature,
        VOCIndex,
        NOxIndex,
        CO2
    };

    // Struct to hold a single measurement value and its type
    struct Measurement {
        MeasurementType type;
        float value;
    };

    // Constructor
    explicit AirQualitySensor(float sensorAltitude = 0.0f)
        : m_sensorAltitude(sensorAltitude)
    {
    }

    // Virtual destructor for proper cleanup
    virtual ~AirQualitySensor() = default;

    // Initialize the sensor
    virtual void Init() = 0;

    // Get the set of measurement types supported by this sensor
    virtual std::set<MeasurementType> GetSupportedMeasurements() const = 0;

    // Read all supported measurements
    // Returns a vector of Measurement structs for all available measurements
    virtual std::vector<Measurement> ReadAllMeasurements() = 0;

    // Start continuous measurement mode (if supported)
    // Returns 0 on success, non-zero on failure
    virtual int StartContinuousMeasurement() = 0;

protected:
    // Set sensor altitude (protected, as it’s implementation-specific)
    virtual int SetSensorAltitude(float sensorAltitude) = 0;

    float m_sensorAltitude;
};