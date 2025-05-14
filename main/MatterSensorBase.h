#pragma once

#include <esp_matter.h>
#include <esp_err.h>
#include <esp_log.h>
#include <common_macros.h>
#include <optional>

using namespace esp_matter;
using namespace esp_matter::endpoint;

class MatterSensorBase {
public:
    // Constructor initializes common members
    MatterSensorBase(node_t* node, const char* tag)
        : m_node(node), m_endpoint(nullptr), m_tag(tag) {}

    virtual ~MatterSensorBase() = default;

    // Pure virtual function for creating the endpoint
    virtual endpoint_t* CreateEndpoint() = 0;

    // Pure virtual function for updating measurements
    virtual void UpdateMeasurements() = 0;

protected:
    // Updates an int16 attribute for the given cluster and attribute IDs
    void UpdateAttributeValueInt16(uint32_t cluster_id, uint32_t attribute_id, int16_t value);

    // Updates a float attribute for the given cluster and attribute IDs
    void UpdateAttributeValueFloat(uint32_t cluster_id, uint32_t attribute_id, float value);

    // Updates the RelativeHumidityMeasurement cluster's MeasuredValue attribute with a scaled value (0.01% units)
    void UpdateRelativeHumidityMeasurementAttributes(std::optional<float> relativeHumidity);

    // Updates the TemperatureMeasurement cluster's MeasuredValue attribute with a scaled value (0.01°C units)
    void UpdateTemperatureMeasurementAttributes(std::optional<float> temperature);

    // Template method to schedule attribute updates on the Matter thread
    template <typename T>
    void ScheduleAttributeUpdate(void (*updateFunc)(T*), T* instance) {
        chip::DeviceLayer::SystemLayer().ScheduleLambda([=]() {
            updateFunc(instance);
        });
    }

    node_t* m_node;
    endpoint_t* m_endpoint;
    const char* m_tag; // For logging with sensor-specific tag
};