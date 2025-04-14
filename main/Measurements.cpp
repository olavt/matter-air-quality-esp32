#include "Measurements.h"
#include "esp_timer.h"
#include <stdexcept>

float getElapsedSeconds()
{
    return static_cast<float>(esp_timer_get_time()) / 1000000.0f;
}

void Measurements::AddId(uint32_t id, size_t averageWindowSizeSeconds, size_t peakWindowSizeSeconds)
{
    m_measurements.emplace(id, MeasuredValues(id, averageWindowSizeSeconds, peakWindowSizeSeconds));
}

void Measurements::AddMeasurement(uint32_t id, float value, float elapsedTimeSeconds)
{
    auto it = m_measurements.find(id);
    it->second.Add(value, elapsedTimeSeconds);
}

void Measurements::AddMeasurementNow(uint32_t id, float value)
{
    AddMeasurement(id, value, getElapsedSeconds());
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

size_t Measurements::GetAverageWindowSizeSeconds(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetAverageWindowSizeSeconds();
}

float Measurements::GetPeak(uint32_t id)
{
    auto it = m_measurements.find(id);
    return it->second.GetPeak();
}

size_t Measurements::GetPeakWindowSizeSeconds(uint32_t id)
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