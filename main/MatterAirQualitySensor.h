#pragma once

#include <esp_matter.h>

#include "sensors/AirQualitySensor.h"
#include "MatterExtendedColorLight.h"
#include "Measurements.h"
#include "MatterSensorBase.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters::AirQuality;

// Represents a Matter Air Quality Sensor device type, compliant with the Matter protocol.
// This class interfaces with a physical air quality sensor to read various environmental
// measurements (e.g., CO2, PM2.5, humidity, temperature) and updates the corresponding
// Matter clusters (e.g., Air Quality (0x002B), Temperature Measurement (0x0402),
// Relative Humidity Measurement (0x0405)) for a specified endpoint on a Matter node.
// It also controls a light endpoint to visualize air quality status.
class MatterAirQualitySensor : public MatterSensorBase
{
    public:

        // Constructs a MatterAirQualitySensor instance, linking it to a Matter node, a physical air quality sensor for environmental measurements (e.g., CO2, PM2.5), and a light endpoint for air quality visualization.
        // @param node The Matter node for attaching the sensor's endpoint.
        // @param airQualitySensor Pointer to the physical air quality sensor.
        // @param lightEndpoint Pointer to the MatterExtendedColorLight for visual status indication.
        MatterAirQualitySensor(node_t* node, AirQualitySensor* airQualitySensor,  MatterExtendedColorLight* lightEndpoint);

        // Creates and configures a Matter endpoint for the air quality sensor, initializing
        // relevant clusters (e.g., Air Quality, Temperature, Humidity) and their attributes.
        // @return Pointer to the created endpoint, or nullptr on failure.
        endpoint_t* CreateEndpoint() override;

        void UpdateMeasurements() override;

    private:

        // Map from MeasurementType to Matter cluster ID
        static const std::unordered_map<AirQualitySensor::MeasurementType, uint32_t> measurementTypeToClusterId;

      AirQualitySensor* m_airQualitySensor;
        MatterExtendedColorLight* m_lightEndpoint;
        Measurements m_measurements;

        void AddRelativeHumidityMeasurementCluster();

        void AddTemperatureMeasurementCluster();

        void AddCarbonDioxideConcentrationMeasurementCluster();

        void AddPm1ConcentrationMeasurementCluster();

        void AddPm25ConcentrationMeasurementCluster();

        void AddPm10ConcentrationMeasurementCluster();

        void AddNitrogenDioxideConcentrationMeasurementCluster();

        void AddTotalVolatileOrganicCompoundsConcentrationMeasurementCluster();

        void AddAirQualityClusterFeatures();

        void SetLightByAirQuality(AirQualityEnum airQuality);

        AirQualityEnum ClassifyAirQualityByCO2();

        AirQualityEnum ClassifyAirQualityByPM10();

        AirQualityEnum ClassifyAirQualityByPM25();

        static void UpdateAirQualityAttributes(MatterAirQualitySensor* airQuality);

};