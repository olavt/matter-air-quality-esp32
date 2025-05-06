/*
 * TemperatureSensor.h
 *
 *  Created on: May 3, 2025
 *      Author: olavt
 */

#pragma once

#include "Sensor.h"
#include <optional>

class TemperatureSensor : public virtual Sensor {
public:
    virtual ~TemperatureSensor() = default;

    // Return-based method for temperature measurement
    virtual std::optional<float> MeasureTemperature() = 0;
};
