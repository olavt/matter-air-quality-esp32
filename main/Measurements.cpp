#include "Measurements.h"
#include "esp_timer.h"
#include <stdexcept>

void Measurements::AddId(uint32_t id, uint32_t averageWindowSizeSeconds, uint32_t peakWindowSizeSeconds)
{
    m_measurements.emplace(id, MeasuredValues(id, averageWindowSizeSeconds, peakWindowSizeSeconds));
}

void Measurements::AddMeasurement(uint32_t id, float value, float elapsedTimeSeconds)
{
    auto it = m_measurements.find(id);
    it->second.Add(value, elapsedTimeSeconds);
}

float Measurements::GetLatest(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetLatest();
}

float Measurements::GetAverage(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetAverage();
}

uint32_t Measurements::GetAverageWindowSizeSeconds(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetAverageWindowSizeSeconds();
}

float Measurements::GetPeak(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetPeak();
}

uint32_t Measurements::GetPeakWindowSizeSeconds(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetPeakWindowSizeSeconds();
}

std::vector<uint32_t> Measurements::GetIds() const
{
    std::vector<uint32_t> ids;
    ids.reserve(m_measurements.size()); // Optimize allocation
    for (const auto& pair : m_measurements) {
        ids.push_back(pair.first);
    }
    return ids;
}