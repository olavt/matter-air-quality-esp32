#include "MatterTemperatureSensor.h"

#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <cmath>

using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "MatterTemperatureSensor";

MatterTemperatureSensor::MatterTemperatureSensor(node_t* node, TemperatureSensor* temperatureSensor)
{
    m_node = node;
    m_temperatureSensor = temperatureSensor;
}

endpoint_t* MatterTemperatureSensor::CreateEndpoint()
{
    ESP_LOGI(TAG, "Creating Endpoint");

    // Create Temperature Endpoint
    esp_matter::endpoint::temperature_sensor::config_t temperature_config;
    m_temperatureEndpoint = esp_matter::endpoint::temperature_sensor::create(m_node, &temperature_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(m_temperatureEndpoint != nullptr, ESP_LOGE(TAG, "Failed to create temperature sensor endpoint"));

    return m_temperatureEndpoint;
}

void MatterTemperatureSensor::UpdateMeasurements()
{
    auto temperature = m_temperatureSensor->MeasureTemperature();
    // Check if the measurement is valid
    if (!temperature.has_value())
    {
        ESP_LOGE(TAG, "MeasureTemperature: Failed to read temperature");
        return;
    }
    
    m_temperatureMeasurement = temperature.value();
    ESP_LOGI(TAG, "MeasureTemperature: %f", m_temperatureMeasurement.value());

    // Need to use ScheduleLambda to execute the updates to the clusters on the Matter thread for thread safety
    chip::DeviceLayer::SystemLayer().ScheduleLambda(
        [
            matterTemperature = this
        ]
        {
            UpdateTemperatureAttributes(matterTemperature);    
        }
    );
}

static void UpdateAttributeValueInt16(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, int16_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_int16(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterTemperatureSensor::UpdateTemperatureAttributes(MatterTemperatureSensor* matterTemperature)
{
    if (!matterTemperature->m_temperatureMeasurement.has_value()) {
        return;
    }

    float temperatureCelsius = matterTemperature->m_temperatureMeasurement.value();
    int16_t reportedTemperature = static_cast<int16_t>(std::round(temperatureCelsius * 100));

    UpdateAttributeValueInt16(
        matterTemperature->m_temperatureEndpoint,
        TemperatureMeasurement::Id,
        TemperatureMeasurement::Attributes::MeasuredValue::Id,
        reportedTemperature);  
}