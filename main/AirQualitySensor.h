#pragma once

class AirQualitySensor
{
public:

  class MeasuredValues
  {
  public:

    float ParticulateMatter1p0 = 0;
    float ParticulateMatter2p5 = 0;
    float ParticulateMatter4p0 = 0;
    float ParticulateMatter10p0 = 0;
    float AmbientHumidity = 0;
    float AmbientTemperature = 0;
    float VOCIndex = 0;
    float NOxIndex = 0;
    float CO2 = 0;
  };

    // Virtual destructor (important for proper cleanup in inheritance)
    virtual ~AirQualitySensor() = default;

    // Pure virtual functions (abstract methods)
    virtual void Init() = 0;
    
    virtual int ReadMeasuredValues(MeasuredValues* measuredValues) = 0;
    
    virtual int SetSensorAltitude(float altitude) = 0;
    
    virtual int StartContiniousMeasurement() = 0;

};