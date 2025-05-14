#include <cmath>
#include "MatterSensorBase.h"

void MatterSensorBase::UpdateAttributeValueInt16(uint32_t cluster_id, uint32_t attribute_id, int16_t value)
{
    if (!m_endpoint) {
        ESP_LOGE(m_tag, "Endpoint not initialized.");
        return;
    }
    uint16_t endpoint_id = endpoint::get_id(m_endpoint);
    esp_matter_attr_val_t val = esp_matter_int16(value);
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterSensorBase::UpdateAttributeValueFloat(uint32_t cluster_id, uint32_t attribute_id, float value)
{
    if (!m_endpoint) {
        ESP_LOGE(m_tag, "Endpoint not initialized.");
        return;
    }
    uint16_t endpoint_id = endpoint::get_id(m_endpoint);
    esp_matter_attr_val_t val = esp_matter_float(value);
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterSensorBase::UpdateRelativeHumidityMeasurementAttributes(std::optional<float> relativeHumidity)
{
    if (!relativeHumidity.has_value()) {
        ESP_LOGE(m_tag, "Relative humidity measurement invalid.");
        return;
    }
    ESP_LOGI(m_tag, "Relative Humidity: %f", relativeHumidity.value());
    uint16_t reportedHumidity = static_cast<uint16_t>(std::round(relativeHumidity.value() * 100));
    UpdateAttributeValueInt16(
        chip::app::Clusters::RelativeHumidityMeasurement::Id,
        chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
        reportedHumidity);
}

void MatterSensorBase::UpdateTemperatureMeasurementAttributes(std::optional<float> temperature)
{
    if (!temperature.has_value()) {
        ESP_LOGE(m_tag, "Temperature measurement invalid.");
        return;
    }
    ESP_LOGI(m_tag, "Temperature: %f", temperature.value());
    int16_t reportedTemperature = static_cast<int16_t>(std::round(temperature.value() * 100));
    UpdateAttributeValueInt16(
        chip::app::Clusters::TemperatureMeasurement::Id,
        chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
        reportedTemperature);
}