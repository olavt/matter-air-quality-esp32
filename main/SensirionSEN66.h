#include "AirQualitySensor.h"

class SensirionSEN66 : public AirQualitySensor
{
public:

  SensirionSEN66(float sensorAltitude = 0.0f)
      : AirQualitySensor(sensorAltitude)
  {
  }

  void Init();
    
  int ReadMeasuredValues(MeasuredValues* measuredValues);

  int StartContinuousMeasurement();

private:

  int SetSensorAltitude(float altitude);

};