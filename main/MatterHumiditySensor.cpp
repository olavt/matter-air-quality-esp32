#include "MatterHumiditySensor.h"

#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <cmath>

using namespace esp_matter::attribute;
using namespace chip::app::Clusters;

static const char *TAG = "MatterHumiditySensor";

MatterHumiditySensor::MatterHumiditySensor(node_t* node, RelativeHumiditySensor* humiditySensor)
        : MatterSensorBase(node, "MatterHumiditySensor"), m_humiditySensor(humiditySensor)
{    
}

endpoint_t*  MatterHumiditySensor::CreateEndpoint()
{
    // Create Humidity Endpoint
    esp_matter::endpoint::humidity_sensor::config_t humidity_config;
    m_endpoint = esp_matter::endpoint::humidity_sensor::create(m_node, &humidity_config, ENDPOINT_FLAG_NONE, NULL);
    ABORT_APP_ON_FAILURE(m_endpoint != nullptr, ESP_LOGE(TAG, "Failed to create humidity sensor endpoint."));

    return m_endpoint;
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
    ScheduleAttributeUpdate(&UpdateAttributes, this);
}

void MatterHumiditySensor::UpdateAttributes(MatterHumiditySensor* matterHumidity)
{
    matterHumidity->UpdateRelativeHumidityMeasurementAttributes(matterHumidity->m_humidityMeasurement);
}