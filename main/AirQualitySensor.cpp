#include "AirQualitySensor.h"
#include <map>

void AirQualitySensor::Init()
{
    SetSensorAltitude(m_sensorAltitude);
}

std::string AirQualitySensor::MeasurementTypeToString(MeasurementType type) {
    static const std::map<MeasurementType, std::string> typeToString = {
        {MeasurementType::ParticulateMatter1p0, "ParticulateMatter1p0"},
        {MeasurementType::ParticulateMatter2p5, "ParticulateMatter2p5"},
        {MeasurementType::ParticulateMatter4p0, "ParticulateMatter4p0"},
        {MeasurementType::ParticulateMatter10p0, "ParticulateMatter10p0"},
        {MeasurementType::AmbientHumidity, "AmbientHumidity"},
        {MeasurementType::AmbientTemperature, "AmbientTemperature"},
        {MeasurementType::VOCIndex, "VOCIndex"},
        {MeasurementType::NOxIndex, "NOxIndex"},
        {MeasurementType::CO2, "CO2"}
    };

    auto it = typeToString.find(type);
    return it->second;
}