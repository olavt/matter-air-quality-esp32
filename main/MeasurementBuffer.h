#pragma once
#include <deque>

class MeasurementBuffer
{
    public:

        MeasurementBuffer(size_t maxSize);

        void AddMeasurement(float value);

        float GetLatest();

        float GetAverage();

    private:

        size_t m_maxSize;
        std::deque<float> m_buffer;
};