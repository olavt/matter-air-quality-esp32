#pragma once

#include <esp_matter.h>

#include "sensors/TemperatureSensor.h"
#include "MatterSensorBase.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters::AirQuality;

// Represents a Matter Temperature Sensor device type, compliant with the Matter protocol.
// This class interfaces with a physical temperature sensor to read temperature measurements
// and updates the corresponding Matter Temperature Measurement cluster (0x0402) attributes
// for a specified endpoint on a Matter node.
class MatterTemperatureSensor : public MatterSensorBase
{
    public:

        // Constructs a MatterTemperatureSensor instance, associating it with a Matter node
        // and a physical temperature sensor.
        // @param node The Matter node to which this sensor's endpoint will be attached.
        // @param temperatureSensor Pointer to the physical temperature sensor providing measurements.
        MatterTemperatureSensor(node_t* node, TemperatureSensor* temperatureSensor);

        // Creates and configures a Matter endpoint for the temperature sensor, initializing
        // the Temperature Measurement cluster (0x0402) and its attributes.
        // @return Pointer to the created endpoint, or nullptr on failure.
        endpoint_t* CreateEndpoint() override;

        // Reads the latest temperature measurement from the physical sensor and updates
        // the Matter Temperature Measurement cluster's MeasuredValue attribute.
        void UpdateMeasurements() override;

    private:

        TemperatureSensor* m_temperatureSensor;
        std::optional<float> m_temperatureMeasurement;

        // Updates the Temperature Measurement cluster's attributes (e.g., MeasuredValue)
        // with the latest temperature reading for the specified MatterTemperatureSensor instance.
        // @param matterTemperature Pointer to the MatterTemperatureSensor instance to update.
        static void UpdateAttributes(MatterTemperatureSensor* matterTemperature);

};
