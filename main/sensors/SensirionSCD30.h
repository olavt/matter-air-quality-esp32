#include "AirQualitySensor.h"
#include <set>
#include <vector>
#include <optional>

class SensirionSCD30 : public AirQualitySensor
{
public:
    // Constructor
    explicit SensirionSCD30(float sensorAltitude = 0.0f)
      : AirQualitySensor(sensorAltitude)
    {
    }

    // Initialize the sensor
    bool Init() override;

    // Get the set of measurement types supported by this sensor
    std::set<Sensor::MeasurementType> GetSupportedMeasurements() const override;

    // Read all supported measurements
    std::vector<AirQualitySensor::Measurement> ReadAllMeasurements() override;

    int ActivateAutomaticSelfCalibration() override;

    // Start continuous measurement mode
    int StartContinuousMeasurement() override;

    int SetAmbientPressure(float ambientPressureKiloPascal) override;

protected:
    // Set sensor altitude
    int SetAltitude(float altitude) override;

};
