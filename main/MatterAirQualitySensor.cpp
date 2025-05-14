#include "MatterAirQualitySensor.h"

#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <math.h>

using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "MatterAirQualitySensor";

const std::unordered_map<AirQualitySensor::MeasurementType, uint32_t> MatterAirQualitySensor::measurementTypeToClusterId = {
    {Sensor::MeasurementType::RelativeHumidity, RelativeHumidityMeasurement::Id},
    {Sensor::MeasurementType::Temperature, TemperatureMeasurement::Id},
    {Sensor::MeasurementType::CO2, CarbonDioxideConcentrationMeasurement::Id},
    {Sensor::MeasurementType::NOx, NitrogenDioxideConcentrationMeasurement::Id},
    {Sensor::MeasurementType::VOC, TotalVolatileOrganicCompoundsConcentrationMeasurement::Id},
    {Sensor::MeasurementType::PM1p0, Pm1ConcentrationMeasurement::Id},
    {Sensor::MeasurementType::PM2p5, Pm25ConcentrationMeasurement::Id},
    {Sensor::MeasurementType::PM10p0, Pm10ConcentrationMeasurement::Id}
};

MatterAirQualitySensor::MatterAirQualitySensor(node_t* node, AirQualitySensor* airQualitySensor, MatterExtendedColorLight* lightEndpoint)
        : MatterSensorBase(node, "MatterAirQualitySensor"), m_airQualitySensor(airQualitySensor), m_lightEndpoint(lightEndpoint)
{
}

endpoint_t* MatterAirQualitySensor::CreateEndpoint()
{
    // Create Air Quality Endpoint
    esp_matter::endpoint::air_quality_sensor::config_t air_quality_config;
    m_endpoint = air_quality_sensor::create(m_node, &air_quality_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(m_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create air quality sensor endpoint"));

    AddAirQualityClusterFeatures();

    // Add Concentration Measurement Clusters
    std::set<AirQualitySensor::MeasurementType> supportedMeasurements = m_airQualitySensor->GetSupportedMeasurements();

    // Add Concentration Measurement Clusters based on supported measurements
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::RelativeHumidity)) {
        AddRelativeHumidityMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::Temperature)) {
        AddTemperatureMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::CO2)) {
        AddCarbonDioxideConcentrationMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::PM1p0)) {
        AddPm1ConcentrationMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::PM2p5)) {
        AddPm25ConcentrationMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::PM10p0)) {
        AddPm10ConcentrationMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::NOx)) {
        AddNitrogenDioxideConcentrationMeasurementCluster();
    }
    if (supportedMeasurements.count(AirQualitySensor::MeasurementType::VOC)) {
        AddTotalVolatileOrganicCompoundsConcentrationMeasurementCluster();
    }

    return m_endpoint;
}

void MatterAirQualitySensor::AddRelativeHumidityMeasurementCluster()
{
    m_measurements.AddId(RelativeHumidityMeasurement::Id, 60, 60);

    esp_matter::cluster::relative_humidity_measurement::config_t relative_humidity_config;
    esp_matter::cluster::relative_humidity_measurement::create(m_endpoint, &relative_humidity_config, CLUSTER_FLAG_SERVER);
}

void MatterAirQualitySensor::AddTemperatureMeasurementCluster()
{
    m_measurements.AddId(TemperatureMeasurement::Id, 60, 60);

    // Add TemperatureMeasurement cluster
    cluster::temperature_measurement::config_t cluster_config;
    cluster::temperature_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);
}

void MatterAirQualitySensor::AddCarbonDioxideConcentrationMeasurementCluster()
{
    m_measurements.AddId(CarbonDioxideConcentrationMeasurement::Id, 3600, 3600);

    cluster::carbon_dioxide_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(CarbonDioxideConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::carbon_dioxide_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);
    
    // Add the NumericMeasurement (MEA) Feature flag    
    cluster::carbon_dioxide_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(CarbonDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm);
    cluster::carbon_dioxide_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::carbon_dioxide_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(CarbonDioxideConcentrationMeasurement::Id);
    cluster::carbon_dioxide_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::carbon_dioxide_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(CarbonDioxideConcentrationMeasurement::Id);
    cluster::carbon_dioxide_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);
}

void MatterAirQualitySensor::AddPm1ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm1ConcentrationMeasurement::Id, 3600, 3600);

    esp_matter::cluster::pm1_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(Pm1ConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::pm1_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag    
    cluster::pm1_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(Pm1ConcentrationMeasurement::MeasurementUnitEnum::kUgm3);
    cluster::pm1_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm1_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(Pm1ConcentrationMeasurement::Id);
    cluster::pm1_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm1_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(Pm1ConcentrationMeasurement::Id);
    cluster::pm1_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);
}

void MatterAirQualitySensor::AddPm25ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm25ConcentrationMeasurement::Id, 3600, 3600);

    esp_matter::cluster::pm25_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(Pm25ConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::pm25_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::pm25_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(Pm25ConcentrationMeasurement::MeasurementUnitEnum::kUgm3);
    cluster::pm25_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm25_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(Pm25ConcentrationMeasurement::Id);
    cluster::pm25_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);   

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm25_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(Pm25ConcentrationMeasurement::Id);
    cluster::pm25_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);
}

void MatterAirQualitySensor::AddPm10ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm10ConcentrationMeasurement::Id, 3600, 3600);

    esp_matter::cluster::pm10_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(Pm10ConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::pm10_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::pm10_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(Pm10ConcentrationMeasurement::MeasurementUnitEnum::kUgm3);
    cluster::pm10_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm10_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(Pm10ConcentrationMeasurement::Id);
    cluster::pm10_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm10_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(Pm10ConcentrationMeasurement::Id);
    cluster::pm10_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);   
}

void MatterAirQualitySensor::AddNitrogenDioxideConcentrationMeasurementCluster()
{
    m_measurements.AddId(NitrogenDioxideConcentrationMeasurement::Id, 3600.0, 3600);

    esp_matter::cluster::nitrogen_dioxide_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(NitrogenDioxideConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::nitrogen_dioxide_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(NitrogenDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm);
    cluster::nitrogen_dioxide_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(NitrogenDioxideConcentrationMeasurement::Id);
    cluster::nitrogen_dioxide_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(NitrogenDioxideConcentrationMeasurement::Id);
    cluster::nitrogen_dioxide_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);  
}

void MatterAirQualitySensor::AddTotalVolatileOrganicCompoundsConcentrationMeasurementCluster()
{
    m_measurements.AddId(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id, 3600, 3600);

    esp_matter::cluster::total_volatile_organic_compounds_concentration_measurement::config_t cluster_config;
    cluster_config.measurement_medium = static_cast<uint8_t>(TotalVolatileOrganicCompoundsConcentrationMeasurement::MeasurementMediumEnum::kAir);
    cluster_t* cluster = esp_matter::cluster::total_volatile_organic_compounds_concentration_measurement::create(m_endpoint, &cluster_config, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    numeric_measurement_config.measurement_unit = static_cast<uint8_t>(TotalVolatileOrganicCompoundsConcentrationMeasurement::MeasurementUnitEnum::kPpm);
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    average_measurement_config.average_measured_value_window = m_measurements.GetAverageWindowSizeSeconds(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id);
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    peak_measurement_config.peak_measured_value_window = m_measurements.GetPeakWindowSizeSeconds(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id);
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);
}

void MatterAirQualitySensor::AddAirQualityClusterFeatures()
{
    cluster_t *cluster = cluster::get(m_endpoint, AirQuality::Id);

    /* Add additional features to the Air Quality cluster */
    cluster::air_quality::feature::fair::add(cluster);
    cluster::air_quality::feature::moderate::add(cluster);
    cluster::air_quality::feature::very_poor::add(cluster);
    cluster::air_quality::feature::extremely_poor::add(cluster);
}

static uint8_t HueDegreesToUInt8(float degrees)
{
    // Wrap degrees to 0-360 using modulo
    float wrapped = fmodf(degrees, 360.0f);

    // Scale to 0-256 and round
    float scaled = (wrapped / 360.0f) * 256.0f;

    // Cast to uint8_t, letting 256 overflow to 0
    return (uint8_t)(scaled + 0.5f);    
}

void MatterAirQualitySensor::SetLightByAirQuality(AirQualityEnum airQuality)
{
    uint8_t saturation = 254; // Full saturation for vivid colors. Note! 255 is reserved and should not be used.
    uint8_t hue = 0;
    float lightLevelPercent = 0.0;

    switch (airQuality) {
        case AirQuality::AirQualityEnum::kGood:
            hue = HueDegreesToUInt8(120.0); // Green
            lightLevelPercent = 10.0;
            break;
        case AirQuality::AirQualityEnum::kFair:
            hue = HueDegreesToUInt8(100.0); // Green-yellow
            lightLevelPercent = 10.0;
            break;
        case AirQuality::AirQualityEnum::kModerate:
            hue = HueDegreesToUInt8(80.0);  // Yellow-green (~80°)
            lightLevelPercent = 15.0;
            break;
        case AirQuality::AirQualityEnum::kPoor:
            hue = HueDegreesToUInt8(60.0);  // Yellow
            lightLevelPercent = 15.0;
            break;
        case AirQuality::AirQualityEnum::kVeryPoor:
            hue = HueDegreesToUInt8(30.0);  // Orange
            lightLevelPercent = 20.0;
            break;
        case AirQuality::AirQualityEnum::kExtremelyPoor:
            hue = HueDegreesToUInt8(0.0);   // Red
            lightLevelPercent = 30.0;
            break;
        case AirQuality::AirQualityEnum::kUnknown:
            hue = 0;     // Neutral (could also reduce saturation)
            lightLevelPercent = 0.0; // Off
            break;
        default:
            ESP_LOGE(TAG, "Unknown air quality enum value");
            return;
    }

    m_lightEndpoint->SetLightOnOff(true);
        m_lightEndpoint->SetLightColorHSV(hue, saturation);
    m_lightEndpoint->SetLightLevelPercent(lightLevelPercent);  
}

AirQualityEnum MatterAirQualitySensor::ClassifyAirQualityByCO2()
{
    uint16_t co2_ppm = m_measurements.GetLatest(CarbonDioxideConcentrationMeasurement::Id);

    if (co2_ppm >= 400 && co2_ppm <= 600) {
        // Fresh air, no noticeable effects; matches outdoor levels.
        return AirQualityEnum::kGood;
    } else if (co2_ppm <= 700) {
        // Still very good, no perceptible impact; minor ventilation decline.
        return AirQualityEnum::kFair;
    } else if (co2_ppm <= 800) {
        // Suboptimal; sensitive individuals might notice slight stuffiness.
        return AirQualityEnum::kModerate;
    } else if (co2_ppm <= 950) {
        // Mild effects possible (e.g., reduced focus); ventilation clearly poor.
        return AirQualityEnum::kPoor;
    } else if (co2_ppm <= 1200) {
        // Discomfort likely (e.g., stuffiness, drowsiness); significant air quality decline.
        return AirQualityEnum::kVeryPoor;
    } else if (co2_ppm > 1200) {
        // Potential health impacts (e.g., fatigue, headaches); unacceptable levels.
        return AirQualityEnum::kExtremelyPoor;
    }
    // Below 400 ppm or invalid readings; sensor error or uninitialized state.
    return AirQualityEnum::kUnknown;
}

AirQualityEnum MatterAirQualitySensor::ClassifyAirQualityByPM10()
{
    uint16_t pm10 = m_measurements.GetAverage(Pm10ConcentrationMeasurement::Id);

    if (pm10 <= 30.0) {
        return AirQualityEnum::kGood;
    } else if (pm10 <= 60.0) {
        return AirQualityEnum::kFair;
    } else if (pm10 <= 120.0) {
        return AirQualityEnum::kModerate;
    } else if (pm10 <= 260.0) {
        return AirQualityEnum::kPoor;
    } else if (pm10 <= 400) {
        return AirQualityEnum::kVeryPoor;
    } else {
        return AirQualityEnum::kExtremelyPoor;
    }
}

AirQualityEnum MatterAirQualitySensor::ClassifyAirQualityByPM25()
{
    uint16_t pm25 = m_measurements.GetAverage(Pm25ConcentrationMeasurement::Id);

    if (pm25 <= 15.0) {
        return AirQualityEnum::kGood;
    } else if (pm25 <= 30.0) {
        return AirQualityEnum::kFair;
    } else if (pm25 <= 50.0) {
        return AirQualityEnum::kModerate;
    } else if (pm25 <= 100.0) {
        return AirQualityEnum::kPoor;
    } else if (pm25 <= 150.0) {
        return AirQualityEnum::kVeryPoor;
    } else {
        return AirQualityEnum::kExtremelyPoor;
    }
}

float getElapsedSeconds()
{
    return static_cast<float>(esp_timer_get_time()) / 1000000.0f;
}

void MatterAirQualitySensor::UpdateMeasurements()
{
   // Read all measurements from the sensor
    std::vector<AirQualitySensor::Measurement> measurements = m_airQualitySensor->ReadAllMeasurements();

    // Check if measurements are empty (indicating an error)
    if (measurements.empty()) {
        ESP_LOGE(TAG, "MeasureAirQuality: sensor->ReadAllMeasurements failed or returned no data");
        return;
    }

    // Process each measurement
    for (const auto& measurement : measurements) {

        // Look up the cluster ID for the measurement type
        auto it = measurementTypeToClusterId.find(measurement.type);

            // Check if the cluster ID was found
        if (it == measurementTypeToClusterId.end()) {
            // Log an error and skip this measurement
            ESP_LOGW(TAG, "MeasureAirQuality: No cluster ID found for measurement type %s",
                    AirQualitySensor::MeasurementTypeToString(measurement.type).c_str());
            continue; // Skip to the next measurement
        }

        uint32_t clusterId = it->second;

        // Log the measurement
        ESP_LOGI(TAG, "MeasureAirQuality: %s: %f",
                    AirQualitySensor::MeasurementTypeToString(measurement.type).c_str(),
                    measurement.value);

        float elapsedSeconds = getElapsedSeconds();

        // Add the measurement to the measurements store
        m_measurements.AddMeasurement(clusterId, measurement.value, elapsedSeconds);
    }

    // Schedule the update of the attributes on the Matter thread
    // This is necessary for thread safety, as the measurements may be updated from a different thread.
    ScheduleAttributeUpdate(&UpdateAirQualityAttributes, this);
}

void MatterAirQualitySensor::UpdateAirQualityAttributes(MatterAirQualitySensor* matterAirQuality)
{
    std::vector<uint32_t> clusterIds = matterAirQuality->m_measurements.GetIds();
    for (uint32_t clusterId : clusterIds) {
        if (clusterId == RelativeHumidityMeasurement::Id)
        {
            matterAirQuality->UpdateRelativeHumidityMeasurementAttributes(
                matterAirQuality->m_measurements.GetLatest(clusterId));
        }
        else if (clusterId == TemperatureMeasurement::Id)
        {
            matterAirQuality->UpdateTemperatureMeasurementAttributes(
                matterAirQuality->m_measurements.GetLatest(clusterId));      
        }
        else
        {
            matterAirQuality->UpdateAttributeValueFloat(
                clusterId,
                0x00000000, // MeasuredValue
                matterAirQuality->m_measurements.GetLatest(clusterId));
    
            matterAirQuality->UpdateAttributeValueFloat(
                clusterId,
                0x00000005, // AverageMeasured Value
                matterAirQuality->m_measurements.GetAverage(clusterId));

            matterAirQuality->UpdateAttributeValueFloat(
                clusterId,
                0x00000003, // PeakMeasured Value
                matterAirQuality->m_measurements.GetPeak(clusterId));
        }
    }

    AirQualityEnum airQualityCO2 = matterAirQuality->ClassifyAirQualityByCO2();
    AirQualityEnum airQualityPM25 = matterAirQuality->ClassifyAirQualityByPM25();
    AirQualityEnum airQualityPM10 = matterAirQuality->ClassifyAirQualityByPM10();

    // Use the worst air quality from CO2, PM2.5, and PM10
    AirQualityEnum airQuality = std::max({airQualityCO2, airQualityPM25, airQualityPM10});
    
    matterAirQuality->UpdateAttributeValueInt16(
        AirQuality::Id,
        AirQuality::Attributes::AirQuality::Id,
        static_cast<int16_t>(airQuality));

    matterAirQuality->SetLightByAirQuality(airQuality);
}