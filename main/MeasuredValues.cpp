#include "MeasuredValues.h"

MeasuredValues::MeasuredValues(size_t maxBufferSize)
    : m_maxBufferSize(maxBufferSize)
{
}

void MeasuredValues::AddMeasurement(uint32_t id, float value)
{
    auto [it, inserted] = m_measurements.emplace(id, MeasurementBuffer(m_maxBufferSize));
    it->second.AddMeasurement(value);
}

float MeasuredValues::GetLatest(uint32_t id)
{
    auto it = m_measurements.find(id);
    if (it == m_measurements.end()) {
        return 0.0f;  // Return 0 if no measurements exist for this substance
    }
    return it->second.GetLatest();
}

float MeasuredValues::GetAverage(uint32_t id)
{
    auto it = m_measurements.find(id);
    if (it == m_measurements.end()) {
        return 0.0f;  // Return 0 if no measurements exist for this substance
    }
    return it->second.GetAverage();
}

std::vector<uint32_t> MeasuredValues::GetIds() const
{
    std::vector<uint32_t> ids;
    ids.reserve(m_measurements.size()); // Optional: optimize by reserving space
    for (const auto& pair : m_measurements) {
        ids.push_back(pair.first);
    }
    return ids;
}