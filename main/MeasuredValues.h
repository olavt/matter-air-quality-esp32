#pragma once
#include <stdint.h>
#include <deque>

class MeasuredValues
{
public:
    MeasuredValues(uint32_t id, uint32_t averageWindowSizeSeconds, uint32_t peakWindowSizeSeconds);

    void Add(float value, float elapsedTimeSeconds);

    // Return the most recent measurement as a single-precision floating-point number.
    float GetLatest();

    // Return the average value of MeasuredValue that has been measured during the averageWindowSizeSeconds.
    float GetAverage();

    uint32_t GetAverageWindowSizeSeconds();

    // Return the maximum value of MeasuredValue that has been measured during the peakWindowSizeSeconds.
    float GetPeak();

    uint32_t GetPeakWindowSizeSeconds();

private:
    uint32_t m_id;
    uint32_t m_averageWindowSizeSeconds;
    uint32_t m_peakWindowSizeSeconds;
    float m_latestValue;
    std::deque<std::pair<float, float>> m_measurements; // Stores (value, elapsedTimeSeconds)
};