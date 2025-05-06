/*
 * AmbientLightSensor.h
 *
 *  Created on: May 4, 2025
 *      Author: olavt
 */

#pragma once

#include "Sensor.h"
#include <optional>

class AmbientLightSensor : public virtual Sensor {
public:
    virtual ~AmbientLightSensor() = default;

    // Return-based method for Ambient Light measurement in Lux
    virtual std::optional<float> MeasureAmbientLight() = 0;
};
