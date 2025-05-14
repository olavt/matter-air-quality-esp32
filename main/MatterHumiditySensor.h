#pragma once

#include <esp_matter.h>
#include "sensors/RelativeHumiditySensor.h"
#include "MatterSensorBase.h"

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters::RelativeHumidityMeasurement;

// Represents a Matter Humidity Sensor device type, compliant with the Matter protocol.
// This class interfaces with a physical humidity sensor to read relative humidity measurements
// and updates the corresponding Matter Relative Humidity Measurement cluster (0x0405) attributes
// for a specified endpoint on a Matter node.
class MatterHumiditySensor : public MatterSensorBase
{

public:

    // Constructs a MatterHumiditySensor instance, associating it with a Matter node
    // and a physical humidity sensor.
    // @param node The Matter node to which this sensor's endpoint will be attached.
    // @param humiditySensor Pointer to the physical humidity sensor providing measurements.
    MatterHumiditySensor(node_t* node, RelativeHumiditySensor* humiditySensor);

    // Creates and configures a Matter endpoint for the humidity sensor, initializing
    // the Relative Humidity Measurement cluster (0x0405) and its attributes.
    // @return Pointer to the created endpoint, or nullptr on failure.
    endpoint_t*  CreateEndpoint() override;

    // Reads the latest humidity measurement from the physical sensor and updates
    // the Matter Relative Humidity Measurement cluster's MeasuredValue attribute.
    void UpdateMeasurements() override;

private:
    
    RelativeHumiditySensor* m_humiditySensor;
    std::optional<float> m_humidityMeasurement;

    // Updates the Relative Humidity Measurement cluster's attributes (e.g., MeasuredValue)
    // with the latest humidity reading for the specified MatterHumiditySensor instance.
    // @param matterHumidity Pointer to the MatterHumiditySensor instance to update.
    static void UpdateAttributes(MatterHumiditySensor* matterHumidity);

};