#include "MatterExtendedColorLight.h"

#include <esp_err.h>
#include <esp_log.h>

using namespace chip::app::Clusters;

static const char *TAG = "MatterExtendedColorLight";

MatterExtendedColorLight::MatterExtendedColorLight(node_t* node, void *priv_data)
{
    m_node = node;
    m_priv_data = priv_data;
}

endpoint_t*  MatterExtendedColorLight::CreateEndpoint()
{
    // Create Extended Color Light Endpoint
    extended_color_light::config_t light_config;
    light_config.color_control.color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;
    light_config.color_control.enhanced_color_mode = (uint8_t)ColorControl::ColorMode::kColorTemperature;

    m_extendedColorLightEndpoint = extended_color_light::create(m_node, &light_config, ENDPOINT_FLAG_NONE, m_priv_data);
    if (m_extendedColorLightEndpoint == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create extended color light endpoint");
        return nullptr;
    }

    AddColorControlFeatures();

    uint16_t light_endpoint_id = endpoint::get_id(m_extendedColorLightEndpoint);

        /* Mark deferred persistence for some attributes that might be changed rapidly */
    attribute_t *current_level_attribute = attribute::get(light_endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::set_deferred_persistence(current_level_attribute);

    attribute_t *current_x_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentX::Id);
    attribute::set_deferred_persistence(current_x_attribute);
    attribute_t *current_y_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::CurrentY::Id);
    attribute::set_deferred_persistence(current_y_attribute);
    attribute_t *color_temp_attribute = attribute::get(light_endpoint_id, ColorControl::Id, ColorControl::Attributes::ColorTemperatureMireds::Id);
    attribute::set_deferred_persistence(color_temp_attribute);

    return m_extendedColorLightEndpoint;
}

void MatterExtendedColorLight::SetLightOnOff(bool on)   
{
    UpdateAttributeValueBool(
        OnOff::Id,
        OnOff::Attributes::OnOff::Id,
        on);
}

void MatterExtendedColorLight::SetLightLevelPercent(float levelPercent)
{
    // For the CurrentLevel attribute:
    // A value of 0x00 SHALL NOT be used.
    // A value of 0x01 SHALL indicate the minimum level that can be attained on a device.
    // A value of 0xFE SHALL indicate the maximum level that can be attained on a device.
    // A value of null SHALL represent an undefined value.
    // All other values are application specific gradations from the minimum to the maximum level. 
    uint8_t level = static_cast<uint8_t>((levelPercent / 100.0f) * 0xFD) + 0x01;

    UpdateAttributeValueUInt8(
        LevelControl::Id,
        LevelControl::Attributes::CurrentLevel::Id,
        level);    
}

void MatterExtendedColorLight::SetLightColorHSV(uint8_t hue, uint8_t saturation)
{
    // Hue represents the color. It ranges from 0 to 360 degrees.
    // 0 degrees = Red
    // 120 degrees = Green
    // 240 degrees = Blue
    // In matter it's representet as a byte with the range 0 to 255.

    ESP_LOGI(TAG, "SetLightColorHSV: CurrentHue=%d CurrentSaturation=%d" , hue, saturation);

    // Update CurrentHue
    UpdateAttributeValueUInt8(
        ColorControl::Id,
        ColorControl::Attributes::CurrentHue::Id,
        hue);

    // Update CurrentSaturation
    UpdateAttributeValueUInt8(
        ColorControl::Id,
        ColorControl::Attributes::CurrentSaturation::Id,
        saturation);
        
    // Update ColorMode to kCurrentHueAndCurrentSaturation
    UpdateAttributeValueUInt8(
        ColorControl::Id,
        ColorControl::Attributes::ColorMode::Id,
        (uint8_t)ColorControl::ColorMode::kCurrentHueAndCurrentSaturation);
}

void MatterExtendedColorLight::AddColorControlFeatures()
{
    cluster_t *cluster = cluster::get(m_extendedColorLightEndpoint, ColorControl::Id);

    // Add the HueSaturation (HS) Feature
    cluster::color_control::feature::hue_saturation::config_t config;
    cluster::color_control::feature::hue_saturation::add(cluster, &config);
}

void MatterExtendedColorLight::UpdateAttributeValueBool(uint32_t cluster_id, uint32_t attribute_id, bool value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(m_extendedColorLightEndpoint);

    esp_matter_attr_val_t val = esp_matter_bool(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

void MatterExtendedColorLight::UpdateAttributeValueUInt8(uint32_t cluster_id, uint32_t attribute_id, uint8_t value)
{
    uint16_t endpoint_id = esp_matter::endpoint::get_id(m_extendedColorLightEndpoint);

    esp_matter_attr_val_t val = esp_matter_uint8(value);

    esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}