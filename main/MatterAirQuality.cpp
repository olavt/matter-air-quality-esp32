#include "MatterAirQuality.h"

#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <math.h>

#include "sensirion_common.h"

using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "MatterAirQuality";

MatterAirQuality::MatterAirQuality(AirQualitySensor* airQualitySensor,  endpoint_t* lightEndpoint)
{
    m_airQualitySensor = airQualitySensor;
    m_lightEndpoint = lightEndpoint;
}

void MatterAirQuality::CreateAirQualityEndpoint(node_t* node)
{
    // Create Air Quality Endpoint
    air_quality_sensor::config_t air_quality_config;
    m_airQualityEndpoint = air_quality_sensor::create(node, &air_quality_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(m_airQualityEndpoint != nullptr, ESP_LOGE(TAG, "Failed to create air quality sensor endpoint"));

    AddAirQualityClusterFeatures();

    // Add Concentration Measurement Clusters
    AddRelativeHumidityMeasurementCluster();
    AddTemperatureMeasurementCluster();
    AddCarbonDioxideConcentrationMeasurementCluster();
    AddPm1ConcentrationMeasurementCluster();
    AddPm25ConcentrationMeasurementCluster();
    AddPm10ConcentrationMeasurementCluster();
    AddNitrogenDioxideConcentrationMeasurementCluster();
    AddTotalVolatileOrganicCompoundsConcentrationMeasurementCluster();

    // Initialize Air Quality Sensor
    m_airQualitySensor->Init();
    m_airQualitySensor->SetSensorAltitude(25.0);
}

void MatterAirQuality::StartMeasurements()
{
    // Initialize LED to a known state
    SetLightOnOff(m_lightEndpoint, false);
    SetLightLevelPercent(m_lightEndpoint, 0.0);
    SetLightColorHSV(m_lightEndpoint, 0, 0);

    int status = m_airQualitySensor->StartContiniousMeasurement();
    ABORT_APP_ON_FAILURE(status == NO_ERROR, ESP_LOGE(TAG, "Air Quality Sensor StartContiniousMeasurement failed."));

    // Setup periodic timer to measure air quality

    esp_timer_create_args_t timer_args = {
        .callback = &MeasureAirQualityTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK, // Run callback in a task (safer for I2C)
        .name = "measure_air_quality_timer",
        .skip_unhandled_events = true, // Skip if previous callback is still running
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &m_timer_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return;
    }
    
    // Start the timer to trigger every 60 seconds (1 minute)
    err = esp_timer_start_periodic(m_timer_handle, MEASUREMENT_SAMPLE_SECONDS * 1000000ULL); // In microseconds
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Air quality timer started");
}

void static UpdateAttributeValueBool(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, bool value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_bool(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void static UpdateAttributeValueUInt8(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, uint8_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_uint8(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void UpdateAttributeValueInt16(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, int16_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_int16(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void UpdateAttributeValueUInt16(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, uint16_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_uint16(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void UpdateAttributeValueUInt32(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, uint32_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_uint32(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

static void UpdateAttributeValueFloat(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, float value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_float(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterAirQuality::AddRelativeHumidityMeasurementCluster()
{
    m_measurements.AddId(RelativeHumidityMeasurement::Id, 60.0, 60.0);

    esp_matter::cluster::relative_humidity_measurement::config_t relative_humidity_config;
    esp_matter::cluster::relative_humidity_measurement::create(m_airQualityEndpoint, &relative_humidity_config, CLUSTER_FLAG_SERVER);
}

void MatterAirQuality::AddTemperatureMeasurementCluster()
{
    m_measurements.AddId(TemperatureMeasurement::Id, 60.0, 60.0);

    // Add TemperatureMeasurement cluster
    cluster::temperature_measurement::config_t temperature_measurement;
    cluster::temperature_measurement::create(m_airQualityEndpoint, &temperature_measurement, CLUSTER_FLAG_SERVER);
}

void MatterAirQuality::AddCarbonDioxideConcentrationMeasurementCluster()
{
    m_measurements.AddId(CarbonDioxideConcentrationMeasurement::Id, 3600, 3600.0);

    cluster::carbon_dioxide_concentration_measurement::config_t co2_measurement;
    cluster_t* cluster = esp_matter::cluster::carbon_dioxide_concentration_measurement::create(m_airQualityEndpoint, &co2_measurement, CLUSTER_FLAG_SERVER);
    
    // Add the NumericMeasurement (MEA) Feature flag    
    cluster::carbon_dioxide_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::carbon_dioxide_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::carbon_dioxide_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::carbon_dioxide_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        CarbonDioxideConcentrationMeasurement::Id,
        CarbonDioxideConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(CarbonDioxideConcentrationMeasurement::Id)  
    );

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::carbon_dioxide_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::carbon_dioxide_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        CarbonDioxideConcentrationMeasurement::Id,
        CarbonDioxideConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(CarbonDioxideConcentrationMeasurement::Id)  
    );

    // Set the MeasurementUnit to PPM
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        CarbonDioxideConcentrationMeasurement::Id,
        CarbonDioxideConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(CarbonDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        CarbonDioxideConcentrationMeasurement::Id,
        CarbonDioxideConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(CarbonDioxideConcentrationMeasurement::MeasurementMediumEnum::kAir));
}

void MatterAirQuality::AddPm1ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm1ConcentrationMeasurement::Id, 3600, 3600.0);

    esp_matter::cluster::pm1_concentration_measurement::config_t pm1_measurement;
    cluster_t* cluster = esp_matter::cluster::pm1_concentration_measurement::create(m_airQualityEndpoint, &pm1_measurement, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag    
    cluster::pm1_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::pm1_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm1_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::pm1_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm1ConcentrationMeasurement::Id,
        Pm1ConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(Pm1ConcentrationMeasurement::Id) 
    );

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm1_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::pm1_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm1ConcentrationMeasurement::Id,
        Pm1ConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(Pm1ConcentrationMeasurement::Id)  
    );

    // Set the MeasurementUnit to Microgram per m3
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm1ConcentrationMeasurement::Id,
        Pm1ConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(Pm1ConcentrationMeasurement::MeasurementUnitEnum::kUgm3));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm1ConcentrationMeasurement::Id,
        Pm1ConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(Pm1ConcentrationMeasurement::MeasurementMediumEnum::kAir));
}

void MatterAirQuality::AddPm25ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm25ConcentrationMeasurement::Id, 3600, 3600.0);

    esp_matter::cluster::pm25_concentration_measurement::config_t pm25_measurement;
    cluster_t* cluster = esp_matter::cluster::pm25_concentration_measurement::create(m_airQualityEndpoint, &pm25_measurement, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::pm25_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::pm25_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm25_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::pm25_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm25ConcentrationMeasurement::Id,
        Pm25ConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(Pm25ConcentrationMeasurement::Id) 
    );    

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm25_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::pm25_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm25ConcentrationMeasurement::Id,
        Pm25ConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(Pm25ConcentrationMeasurement::Id)  
    );    

    // Set the MeasurementUnit to Microgram per m3
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm25ConcentrationMeasurement::Id,
        Pm25ConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(Pm25ConcentrationMeasurement::MeasurementUnitEnum::kUgm3));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm25ConcentrationMeasurement::Id,
        Pm25ConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(Pm25ConcentrationMeasurement::MeasurementMediumEnum::kAir));    
}

void MatterAirQuality::AddPm10ConcentrationMeasurementCluster()
{
    m_measurements.AddId(Pm10ConcentrationMeasurement::Id, 3600, 3600.0);

    esp_matter::cluster::pm10_concentration_measurement::config_t pm10_measurement;
    cluster_t* cluster = esp_matter::cluster::pm10_concentration_measurement::create(m_airQualityEndpoint, &pm10_measurement, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::pm10_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::pm10_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::pm10_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::pm10_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);


    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm10ConcentrationMeasurement::Id,
        Pm10ConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(Pm10ConcentrationMeasurement::Id) 
    );

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::pm10_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::pm10_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        Pm10ConcentrationMeasurement::Id,
        Pm10ConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(Pm10ConcentrationMeasurement::Id)  
    ); 

    // Set the MeasurementUnit to Microgram per m3
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm10ConcentrationMeasurement::Id,
        Pm10ConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(Pm10ConcentrationMeasurement::MeasurementUnitEnum::kUgm3));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        Pm10ConcentrationMeasurement::Id,
        Pm10ConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(Pm10ConcentrationMeasurement::MeasurementMediumEnum::kAir));    
}

void MatterAirQuality::AddNitrogenDioxideConcentrationMeasurementCluster()
{
    m_measurements.AddId(NitrogenDioxideConcentrationMeasurement::Id, 3600.0, 3600.0);

    esp_matter::cluster::nitrogen_dioxide_concentration_measurement::config_t nox_measurement;
    cluster_t* cluster = esp_matter::cluster::nitrogen_dioxide_concentration_measurement::create(m_airQualityEndpoint, &nox_measurement, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::nitrogen_dioxide_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::nitrogen_dioxide_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        NitrogenDioxideConcentrationMeasurement::Id,
        NitrogenDioxideConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(NitrogenDioxideConcentrationMeasurement::Id)
    );

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::nitrogen_dioxide_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::nitrogen_dioxide_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        NitrogenDioxideConcentrationMeasurement::Id,
        NitrogenDioxideConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(NitrogenDioxideConcentrationMeasurement::Id)  
    );

    // Set the MeasurementUnit to PPM
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        NitrogenDioxideConcentrationMeasurement::Id,
        NitrogenDioxideConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(NitrogenDioxideConcentrationMeasurement::MeasurementUnitEnum::kPpm));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        NitrogenDioxideConcentrationMeasurement::Id,
        NitrogenDioxideConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(NitrogenDioxideConcentrationMeasurement::MeasurementMediumEnum::kAir));    
}

void MatterAirQuality::AddTotalVolatileOrganicCompoundsConcentrationMeasurementCluster()
{
    m_measurements.AddId(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id, 3600.0, 3600.0);

    esp_matter::cluster::total_volatile_organic_compounds_concentration_measurement::config_t voc_measurement;
    cluster_t* cluster = esp_matter::cluster::total_volatile_organic_compounds_concentration_measurement::create(m_airQualityEndpoint, &voc_measurement, CLUSTER_FLAG_SERVER);

    // Add the NumericMeasurement (MEA) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::numeric_measurement::config_t numeric_measurement_config;
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::numeric_measurement::add(cluster, &numeric_measurement_config);

    // Add the AverageMeasurement (AVG) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::average_measurement::config_t average_measurement_config;
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::average_measurement::add(cluster, &average_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Attributes::AverageMeasuredValueWindow::Id,
        m_measurements.GetAverageWindowSizeSeconds(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id) 
    );

    // Add the PeakMeasurement (PEA) Feature flag
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::peak_measurement::config_t peak_measurement_config;
    cluster::total_volatile_organic_compounds_concentration_measurement::feature::peak_measurement::add(cluster, &peak_measurement_config);

    UpdateAttributeValueUInt32(
        m_airQualityEndpoint,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Attributes::PeakMeasuredValueWindow::Id,
        m_measurements.GetPeakWindowSizeSeconds(TotalVolatileOrganicCompoundsConcentrationMeasurement::Id)  
    ); 

    // Set the MeasurementUnit to PPM
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Attributes::MeasurementUnit::Id,
        static_cast<int16_t>(TotalVolatileOrganicCompoundsConcentrationMeasurement::MeasurementUnitEnum::kPpm));

    // Set the MeasurementMedium to Air
    UpdateAttributeValueUInt8(
        m_airQualityEndpoint,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Attributes::MeasurementMedium::Id,
        static_cast<int16_t>(TotalVolatileOrganicCompoundsConcentrationMeasurement::MeasurementMediumEnum::kAir));
}

void MatterAirQuality::AddAirQualityClusterFeatures()
{
    cluster_t *cluster = cluster::get(m_airQualityEndpoint, AirQuality::Id);

    /* Add additional features to the Air Quality cluster */
    cluster::air_quality::feature::fair::add(cluster);
    cluster::air_quality::feature::moderate::add(cluster);
    cluster::air_quality::feature::very_poor::add(cluster);
    cluster::air_quality::feature::extremely_poor::add(cluster);
}

void MatterAirQuality::SetLightOnOff(endpoint_t* lightEndpoint, bool on)
{
    UpdateAttributeValueBool(
        lightEndpoint,
        OnOff::Id,
        OnOff::Attributes::OnOff::Id,
        on);
}

void MatterAirQuality::SetLightLevelPercent(endpoint_t* lightEndpoint, float levelPercent)
{
    // For the CurrentLevel attribute:
    // A value of 0x00 SHALL NOT be used.
    // A value of 0x01 SHALL indicate the minimum level that can be attained on a device.
    // A value of 0xFE SHALL indicate the maximum level that can be attained on a device.
    // A value of null SHALL represent an undefined value.
    // All other values are application specific gradations from the minimum to the maximum level. 
    uint8_t level = static_cast<uint8_t>((levelPercent / 100.0f) * 0xFD) + 0x01;

    UpdateAttributeValueUInt8(
        lightEndpoint,
        LevelControl::Id,
        LevelControl::Attributes::CurrentLevel::Id,
        level);    
}

void MatterAirQuality::SetLightColorHSV(endpoint_t* lightEndpoint, uint8_t hue, uint8_t saturation)
{
    // Hue represents the color. It ranges from 0 to 360 degrees.
    // 0 degrees = Red
    // 120 degrees = Green
    // 240 degrees = Blue
    // In matter it's representet as a byte with the range 0 to 255.

    ESP_LOGI(TAG, "SetLightColorHSV: CurrentHue=%d CurrentSaturation=%d" , hue, saturation); 

    UpdateAttributeValueUInt8(
        lightEndpoint,
        ColorControl::Id,
        ColorControl::Attributes::CurrentHue::Id,
        hue);

    UpdateAttributeValueUInt8(
        lightEndpoint,
        ColorControl::Id,
        ColorControl::Attributes::CurrentSaturation::Id,
        saturation);    
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

void MatterAirQuality::SetLightByAirQuality(endpoint_t* lightEndpoint, AirQualityEnum airQuality)
{
    uint8_t saturation = 254; // Full saturation for vivid colors. Note! 255 is reserved and should not be used.
    uint8_t hue = 0;
    float level = 0.0;

    switch (airQuality) {
        case AirQuality::AirQualityEnum::kGood:
            hue = HueDegreesToUInt8(120.0); // Green
            level = 10.0;
            break;
        case AirQuality::AirQualityEnum::kFair:
            hue = HueDegreesToUInt8(100.0); // Green-yellow
            level = 10.0;
            break;
        case AirQuality::AirQualityEnum::kModerate:
            hue = HueDegreesToUInt8(80.0);  // Yellow-green (~80°)
            level = 15.0;
            break;
        case AirQuality::AirQualityEnum::kPoor:
            hue = HueDegreesToUInt8(60.0);  // Yellow
            level = 15.0;
            break;
        case AirQuality::AirQualityEnum::kVeryPoor:
            hue = HueDegreesToUInt8(30.0);  // Orange
            level = 20.0;
            break;
        case AirQuality::AirQualityEnum::kExtremelyPoor:
            hue = HueDegreesToUInt8(0.0);   // Red
            level = 30.0;
            break;
        case AirQuality::AirQualityEnum::kUnknown:
            hue = 0;     // Neutral (could also reduce saturation)
            level = 0.0; // Off
            break;
        default:
            ESP_LOGE(TAG, "Unknown air quality enum value");
            return;
    }
    
    SetLightOnOff(lightEndpoint, true);
    SetLightColorHSV(lightEndpoint, hue, saturation);
    SetLightLevelPercent(lightEndpoint, level);   
}

AirQualityEnum MatterAirQuality::ClassifyAirQualityByCO2()
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

AirQualityEnum MatterAirQuality::ClassifyAirQualityByPM10()
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

AirQualityEnum MatterAirQuality::ClassifyAirQualityByPM25()
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

void MatterAirQuality::UpdateAirQualityAttributes(endpoint_t* airQualityEndpoint, endpoint_t* lightEndpoint, MatterAirQuality* matterAirQuality)
{
    std::vector<uint32_t> clusterIds = matterAirQuality->m_measurements.GetIds();
    for (uint32_t clusterId : clusterIds) {
        if (clusterId == RelativeHumidityMeasurement::Id)
        {
            UpdateAttributeValueInt16(
                airQualityEndpoint,
                RelativeHumidityMeasurement::Id,
                RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
                matterAirQuality->m_measurements.GetLatest(clusterId) * 100);
        }
        else if (clusterId == TemperatureMeasurement::Id)
        {
            UpdateAttributeValueInt16(
                airQualityEndpoint,
                TemperatureMeasurement::Id,
                TemperatureMeasurement::Attributes::MeasuredValue::Id,
                matterAirQuality->m_measurements.GetLatest(clusterId) * 100);        
        }
        else
        {
            UpdateAttributeValueFloat(
                airQualityEndpoint,
                clusterId,
                0x0000, // Measured Value
                matterAirQuality->m_measurements.GetLatest(clusterId));
    
            UpdateAttributeValueFloat(
                airQualityEndpoint,
                clusterId,
                0x0005, // Average Measured Value
                matterAirQuality->m_measurements.GetAverage(clusterId));

            UpdateAttributeValueFloat(
                airQualityEndpoint,
                clusterId,
                0x0003, // Peak Measured Value
                matterAirQuality->m_measurements.GetPeak(clusterId));
        }
    }

    AirQualityEnum airQualityCO2 = matterAirQuality->ClassifyAirQualityByCO2();
    AirQualityEnum airQualityPM25 = matterAirQuality->ClassifyAirQualityByPM25();
    AirQualityEnum airQualityPM10 = matterAirQuality->ClassifyAirQualityByPM10();

    // Use the worst air quality from CO2, PM2.5, and PM10
    AirQualityEnum airQuality = std::max({airQualityCO2, airQualityPM25, airQualityPM10});
    
    UpdateAttributeValueInt16(
        airQualityEndpoint,
        AirQuality::Id,
        AirQuality::Attributes::AirQuality::Id,
        static_cast<int16_t>(airQuality));

    SetLightByAirQuality(lightEndpoint, airQuality);
}

void MatterAirQuality::MeasureAirQuality()
{
    AirQualitySensor* sensor = m_airQualitySensor;

    AirQualitySensor::MeasuredValues* measuredValues = new AirQualitySensor::MeasuredValues();

    int status = sensor->ReadMeasuredValues(measuredValues);
    if (status != NO_ERROR) {
        ESP_LOGE(TAG, "MeasureAirQuality: sensor->ReadMeasuredValues failed with status=%d", status);
        return;    
    };

    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: AmbientHumidity: %f", measuredValues->AmbientHumidity);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: AmbientTemperature: %f", measuredValues->AmbientTemperature);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: CO2: %f", measuredValues->CO2);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: NOx: %f", measuredValues->NOxIndex);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: VOC: %f", measuredValues->VOCIndex);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: PM1: %f", measuredValues->ParticulateMatter1p0);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: PM2.5: %f", measuredValues->ParticulateMatter2p5);
    ESP_LOGI(TAG, "MeasureAirQualityTimerCallback: PM10: %f", measuredValues->ParticulateMatter10p0);

    m_measurements.AddMeasurementNow(
        RelativeHumidityMeasurement::Id,
        measuredValues->AmbientHumidity
    );

    m_measurements.AddMeasurementNow(
        TemperatureMeasurement::Id,
        measuredValues->AmbientTemperature
    );

    m_measurements.AddMeasurementNow(
        CarbonDioxideConcentrationMeasurement::Id,
        measuredValues->CO2
    );

    m_measurements.AddMeasurementNow(
        TotalVolatileOrganicCompoundsConcentrationMeasurement::Id,
        measuredValues->VOCIndex
    );

    m_measurements.AddMeasurementNow(
        NitrogenDioxideConcentrationMeasurement::Id,
        measuredValues->NOxIndex);

    m_measurements.AddMeasurementNow(
    Pm1ConcentrationMeasurement::Id,
    measuredValues->ParticulateMatter1p0
    );

    m_measurements.AddMeasurementNow(
        Pm25ConcentrationMeasurement::Id,
        measuredValues->ParticulateMatter2p5
    );

    m_measurements.AddMeasurementNow(
        Pm10ConcentrationMeasurement::Id,
        measuredValues->ParticulateMatter10p0
    );
}

// Timer callback to measure air quality
void MatterAirQuality::MeasureAirQualityTimerCallback(void *arg)
{
    MatterAirQuality* airQuality = static_cast<MatterAirQuality*>(arg);

    airQuality->MeasureAirQuality();
    
    // Need to use ScheduleLambda to execute the updates to the clusters on the Matter thread for thread safety
    chip::DeviceLayer::SystemLayer().ScheduleLambda(
        [
            airQualityEndpoint = airQuality->m_airQualityEndpoint,
            lightEndpoint = airQuality->m_lightEndpoint,
            airQuality
        ]
        {
            UpdateAirQualityAttributes(airQualityEndpoint, lightEndpoint, airQuality);    
        }
    );

}