#pragma once
#include <stdint.h>
#include <deque>

class MeasuredValues
{
public:
    MeasuredValues(uint32_t id, size_t averageWindowSizeSeconds, size_t peakWindowSizeSeconds);

    void Add(float value, float elapsedTimeSeconds);

    // Return the most recent measurement as a single-precision floating-point number.
    float GetLatest();

    // Return the average value of MeasuredValue that has been measured during the averageWindowSizeSeconds.
    float GetAverage();

    size_t GetAverageWindowSizeSeconds();

    // Return the maximum value of MeasuredValue that has been measured during the peakWindowSizeSeconds.
    float GetPeak();

    size_t GetPeakWindowSizeSeconds();

private:
    uint32_t m_id;
    size_t m_averageWindowSizeSeconds;
    size_t m_peakWindowSizeSeconds;
    float m_latestValue;
    std::deque<std::pair<float, float>> m_measurements; // Stores (value, elapsedTimeSeconds)
};