#pragma once

#include <vector>
#include <esp_matter.h>

#include "sensors/AirQualitySensor.h"
#include "Measurements.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters::AirQuality;

class MatterAirQuality
{
    public:

        MatterAirQuality(node_t* node, AirQualitySensor* airQualitySensor,  endpoint_t* lightEndpoint);

        void CreateEndpoint();
        
        void StartMeasurements();

        void MeasureAirQuality();

    private:

        static constexpr uint32_t MEASUREMENT_SAMPLE_SECONDS = 60;

        // Map from MeasurementType to Matter cluster ID
        static const std::unordered_map<AirQualitySensor::MeasurementType, uint32_t> measurementTypeToClusterId;

        node_t* m_node;
        endpoint_t* m_lightEndpoint;
        endpoint_t* m_airQualityEndpoint;
        AirQualitySensor* m_airQualitySensor;
        esp_timer_handle_t m_timer_handle;
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

        static void MeasureAirQualityTimerCallback(void *arg);

        static void SetLightOnOff(endpoint_t* lightEndpoint, bool on);

        static void SetLightLevelPercent(endpoint_t* lightEndpoint, float levelPercent);

        static void SetLightColorHSV(endpoint_t* lightEndpoint, uint8_t hue, uint8_t saturation);

        static void SetLightByAirQuality(endpoint_t* lightEndpoint, AirQualityEnum airQuality);

        AirQualityEnum ClassifyAirQualityByCO2();

        AirQualityEnum ClassifyAirQualityByPM10();

        AirQualityEnum ClassifyAirQualityByPM25();

        static void UpdateAirQualityAttributes(
            endpoint_t* airQualityEndpoint,
            endpoint_t* lightEndpoint,
            MatterAirQuality* airQuality);

};