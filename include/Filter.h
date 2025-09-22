#pragma once

#include <Arduino.h>
#include <deque>

class Filter
{
private:
    const size_t maxSize = 5;
    const uint32_t threshold = 180;
    std::deque<uint32_t> values;

public:
    Filter() {};
    Filter(size_t maxSize, uint32_t threshold) : maxSize(maxSize), threshold(threshold) {};
    uint32_t CheckValue(uint32_t value);
};