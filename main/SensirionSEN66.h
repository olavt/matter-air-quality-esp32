#include "AirQualitySensor.h"

class SensirionSEN66 : public AirQualitySensor
{
public:

  void Init();
    
  int ReadMeasuredValues(MeasuredValues* measuredValues);

  int SetSensorAltitude(float altitude);

  int StartContiniousMeasurement();

};