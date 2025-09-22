#include "Filter.h"

static uint32_t GetMean(const std::deque<uint32_t> &values);

uint32_t Filter::CheckValue(uint32_t value)
{
    // 若当前值与已有值的差值超过阈值，则认为当前值不可靠，输出现有值的均值
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (abs((int32_t)(value - values[i])) > threshold)
        {
            return GetMean(values);
        }
    }

    // 如果是可靠值，加入列表，返回当前值
    if (values.size() >= maxSize)
    {
        values.pop_front();
    }
    values.push_back(value);
    return value;
}

uint32_t GetMean(const std::deque<uint32_t> &values)
{
    if (values.empty())
        return 0;

    uint64_t sum = 0;
    for (const auto &val : values)
    {
        sum += val;
    }
    return static_cast<uint32_t>(sum / values.size());
}
