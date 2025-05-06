/*
 * Sensor.cpp
 *
 *  Created on: May 2, 2025
 *      Author: olavt
 */

#include "Sensor.h"

#include <map>

std::string Sensor::MeasurementTypeToString(MeasurementType measurementType) {
    static const std::map<MeasurementType, std::string> typeToString = {
        {MeasurementType::AmbientLight, "AmbientLight"},
        {MeasurementType::BarometricPressure, "BarometricPressure"},
        {MeasurementType::CO2, "CO2"},
        {MeasurementType::NOx, "NOx"},
        {MeasurementType::PM1p0, "PM1"},
        {MeasurementType::PM2p5, "PM25"},
        {MeasurementType::PM4p0, "PM4"},
        {MeasurementType::PM10p0, "PM10"},
        {MeasurementType::RelativeHumidity, "Humidity"},
        {MeasurementType::Temperature, "Temperature"},
        {MeasurementType::VOC, "VOC"}
    };

    auto it = typeToString.find(measurementType);
    return it->second;
}
