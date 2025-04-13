#include "MeasurementBuffer.h"

MeasurementBuffer::MeasurementBuffer(size_t maxSize)
    : m_maxSize(maxSize)
{   
}

void MeasurementBuffer::AddMeasurement(float value)
{
    if (m_buffer.size() >= m_maxSize) {
        m_buffer.pop_front();    // Remove it
    }
    m_buffer.push_back(value);   // Add new value to the back
}

float MeasurementBuffer::GetLatest()
{
    if (m_buffer.empty()) {
        return 0.0f;  // Return 0 if buffer is empty
    }
    return m_buffer.back();  // Return the most recent measurement
}

float MeasurementBuffer::GetAverage()
{
    if (m_buffer.empty()) {
        return 0.0f;  // Return 0 if buffer is empty to avoid division by zero
    }

    float sum = 0.0f;
    for (float value : m_buffer) {
        sum += value;
    }
    
    return sum / static_cast<float>(m_buffer.size());
}