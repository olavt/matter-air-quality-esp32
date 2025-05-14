#include "MatterHumiditySensor.h"

#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <cmath>

using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "MatterHumiditySensor";

MatterHumiditySensor::MatterHumiditySensor(node_t* node, RelativeHumiditySensor* humiditySensor)
{
    m_node = node;
    m_humiditySensor = humiditySensor;
}

endpoint_t*  MatterHumiditySensor::CreateEndpoint()
{
    // Create Humidity Endpoint
    esp_matter::endpoint::humidity_sensor::config_t humidity_config;
    m_humidityEndpoint = esp_matter::endpoint::humidity_sensor::create(m_node, &humidity_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(m_humidityEndpoint != nullptr, ESP_LOGE(TAG, "Failed to create humidity sensor endpoint"));

    return m_humidityEndpoint;
}

void MatterHumiditySensor::UpdateMeasurements()
{
    auto relativeHumidity = m_humiditySensor->MeasureRelativeHumidity();
    // Check if the measurement is valid
    if (!relativeHumidity.has_value())
    {
        ESP_LOGE(TAG, "MeasureRelativeHumidity: Failed to read humidity");
        return;
    }

    m_humidityMeasurement = relativeHumidity.value();
    ESP_LOGI(TAG, "MeasureRelativeHumidity: %f", m_humidityMeasurement.value());

    // Need to use ScheduleLambda to execute the updates to the clusters on the Matter thread for thread safety
    chip::DeviceLayer::SystemLayer().ScheduleLambda(
        [
            matterHumidity = this
        ]
        {
            UpdateRelativeHumidityAttributes(matterHumidity);    
        }
    );
}

static void UpdateAttributeValueInt16(endpoint_t* endpoint, uint32_t cluster_id, uint32_t attribute_id, int16_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(endpoint);

    esp_matter_attr_val_t val = esp_matter_int16(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterHumiditySensor::UpdateRelativeHumidityAttributes(MatterHumiditySensor* matterHumidity)
{
    if (!matterHumidity->m_humidityMeasurement.has_value()) {
        return;
    }

    float humidityPercent = matterHumidity->m_humidityMeasurement.value();

    uint16_t reportedHumidity = static_cast<uint16_t>(std::round(humidityPercent * 100.0f));
    
    UpdateAttributeValueInt16(
        matterHumidity->m_humidityEndpoint,
        RelativeHumidityMeasurement::Id,
        RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
        reportedHumidity);
}