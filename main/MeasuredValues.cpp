#include "MeasuredValues.h"
#include <deque>
#include <algorithm>
#include <stdexcept>

MeasuredValues::MeasuredValues(uint32_t id, uint32_t averageWindowSizeSeconds, uint32_t peakWindowSizeSeconds)
    : m_id(id)
    , m_averageWindowSizeSeconds(averageWindowSizeSeconds)
    , m_peakWindowSizeSeconds(peakWindowSizeSeconds)
    , m_latestValue(0.0f)
{
}

void MeasuredValues::Add(float value, float elapsedTimeSeconds)
{
    // Store the new measurement with its timestamp
    m_measurements.emplace_back(value, elapsedTimeSeconds);
    m_latestValue = value;

    // Remove measurements outside the larger of the two windows
    float oldestRelevantTime = elapsedTimeSeconds - std::max(m_averageWindowSizeSeconds, m_peakWindowSizeSeconds);
    while (!m_measurements.empty() && m_measurements.front().second < oldestRelevantTime) {
        m_measurements.pop_front();
    }
}

float MeasuredValues::GetLatest()
{
    return m_latestValue;
}

float MeasuredValues::GetAverage()
{
    if (m_measurements.empty()) {
        return 0.0f;
    }

    float sum = 0.0f;
    size_t count = 0;
    float minTime = m_measurements.back().second - m_averageWindowSizeSeconds;

    // Sum values within the average window
    for (const auto& measurement : m_measurements) {
        if (measurement.second >= minTime) {
            sum += measurement.first;
            count++;
        }
    }

    return count > 0 ? sum / count : 0.0f;
}

uint32_t MeasuredValues::GetAverageWindowSizeSeconds()
{
    return m_averageWindowSizeSeconds;
}

float MeasuredValues::GetPeak()
{
    if (m_measurements.empty()) {
        return 0.0f;
    }

    float peak = m_measurements.back().first;
    float minTime = m_measurements.back().second - m_peakWindowSizeSeconds;

    // Find max value within the peak window
    for (const auto& measurement : m_measurements) {
        if (measurement.second >= minTime) {
            peak = std::max(peak, measurement.first);
        }
    }

    return peak;
}

uint32_t MeasuredValues::GetPeakWindowSizeSeconds()
{
    return m_peakWindowSizeSeconds;
}