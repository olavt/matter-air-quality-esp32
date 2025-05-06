/*
 * RelativeHumiditySensor.h
 *
 *  Created on: May 3, 2025
 *      Author: olavt
 */

#pragma once

#include "Sensor.h"
#include <optional>

class RelativeHumiditySensor : public virtual Sensor {
public:
    virtual ~RelativeHumiditySensor() = default;

    // Return-based method for humidity measurement
    virtual std::optional<float> MeasureRelativeHumidity() = 0;
};
