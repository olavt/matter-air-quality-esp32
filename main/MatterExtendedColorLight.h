#pragma once

#include <esp_matter.h>

using namespace esp_matter;
using namespace esp_matter::endpoint;

class MatterExtendedColorLight {

public:

    MatterExtendedColorLight(node_t* node, void *priv_data);

    // Creates and configures a Matter endpoint for the extended color light.
    // @return Pointer to the created endpoint, or nullptr on failure.
    endpoint_t*  CreateEndpoint();

    void SetLightOnOff(bool on);

    void SetLightLevelPercent(float levelPercent);

    void SetLightColorHSV(uint8_t hue, uint8_t saturation);

private:

    node_t* m_node;
    endpoint_t* m_extendedColorLightEndpoint;
    void* m_priv_data;

    void AddColorControlFeatures();

    void UpdateAttributeValueBool(uint32_t cluster_id, uint32_t attribute_id, bool value);

    void UpdateAttributeValueUInt8(uint32_t cluster_id, uint32_t attribute_id, uint8_t value);

};