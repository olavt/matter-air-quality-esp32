#include "AirQualitySensor.h"
#include <map>

bool AirQualitySensor::Init()
{
    SetAltitude(m_sensorAltitude);

    return true;
}
