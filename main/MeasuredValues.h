#pragma once
#include <stdint.h>
#include <unordered_map>
#include <vector>
#include "MeasurementBuffer.h"

class MeasuredValues
{

public:

    MeasuredValues(size_t maxBufferSize);

    void AddMeasurement(uint32_t id, float value);

    float GetLatest(uint32_t id);

    float GetAverage(uint32_t id);

    std::vector<uint32_t> GetIds() const;

private:
    size_t m_maxBufferSize;
    std::unordered_map<uint32_t, MeasurementBuffer> m_measurements;
};