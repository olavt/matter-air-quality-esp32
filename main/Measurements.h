#pragma once
#include "MeasuredValues.h"
#include <stdint.h>
#include <map>
#include <vector>

class Measurements
{
public:
    // Constructor (default, no initialization needed)
    Measurements() = default;

    // Add an ID with its window sizes
    void AddId(uint32_t id, size_t averageWindowSizeSeconds, size_t peakWindowSizeSeconds);

    // Add a measurement for a specific ID
    void AddMeasurement(uint32_t id, float value, float elapsedTimeSeconds);

    // Add a measurement for a specific ID using current elapsed time
    void AddMeasurementNow(uint32_t id, float value);

    // Get the latest measurement for an ID
    float GetLatest(uint32_t id);

    // Get the average measurement for an ID over its window
    float GetAverage(uint32_t id);

    // Get the average window size for an ID
    size_t GetAverageWindowSizeSeconds(uint32_t id);

    // Get the peak measurement for an ID over its window
    float GetPeak(uint32_t id);

    // Get the peak window size for an ID
    size_t GetPeakWindowSizeSeconds(uint32_t id);

    std::vector<uint32_t> GetIds() const;

private:
    std::map<uint32_t, MeasuredValues> m_measurements; // Maps ID to its MeasuredValues
};