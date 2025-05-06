/*
 * BarometricPressureSensor.h
 *
 *  Created on: May 4, 2025
 *      Author: olavt
 */

#pragma once

#include "Sensor.h"
#include <optional>

class BarometricPressureSensor : public virtual Sensor {
public:
    virtual ~BarometricPressureSensor() = default;

    // Return-based method for barometric pressure measurement
    virtual std::optional<float> MeasureBarometricPressure() = 0;
};
