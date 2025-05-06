/*
 * Sensor.h
 *
 *  Created on: May 2, 2025
 *      Author: olavt
 */

#pragma once

#include <set>
#include <string>
#include <vector>

class Sensor
{

public:

  enum class MeasurementType {
    AmbientLight,
    BarometricPressure,
    CO2,
    NOx,
    PM1p0,
    PM2p5,
    PM4p0,
    PM10p0,
    RelativeHumidity,
    Temperature,
    VOC
  };

  // Struct to hold a single measurement value and its type
  struct Measurement {
      MeasurementType type;
      float value;
  };

  // Initialize the sensor
  virtual bool Init() = 0;

  // Read all supported measurements
  // Returns a vector of Measurement structs for all available measurements
  virtual std::vector<Measurement> ReadAllMeasurements() = 0;

  // Convert SensorType to string
  static std::string MeasurementTypeToString(MeasurementType measurementType);

};
