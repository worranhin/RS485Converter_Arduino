#include "MedianFilter.h"

MedianFilter::MedianFilter(uint8_t size) {
  _maxSize = size;
  _rawData = new uint32_t[size];
}

/// @brief add a value to the filter manager, will overwrite the oldest value
/// @param value
void MedianFilter::addValue(uint32_t value) {
  if (_size != 0 && abs((int)(value - this->getMedian())) > 500)
    return;
  _rawData[_index] = value;
  _index = (_index + 1) % _maxSize;
  _size = _size < _maxSize ? _size + 1 : _maxSize;
  _hasCache = false;
}

/// @brief get the median of the values
/// @return
uint32_t MedianFilter::getMedian() {
  if (_size == 0) {
    return 0xffffffff;
  }

  if (_hasCache) {
    return _cache;
  }

  uint32_t* sortedData = new uint32_t[_size];
  for (uint8_t i = 0; i < _size; i++) {
    sortedData[i] = _rawData[i];
  }

  // 排序
  for (uint8_t i = 0; i < _size - 1; i++) {
    for (uint8_t j = 0; j < _size - i - 1; j++) {
      if (sortedData[j] > sortedData[j + 1]) {
        uint32_t temp = sortedData[j];
        sortedData[j] = sortedData[j + 1];
        sortedData[j + 1] = temp;
      }
    }
  }

  // 取中位数
  uint32_t median;
  if (_size % 2 == 0) {
    median = (sortedData[_size / 2] + sortedData[_size / 2 - 1]) / 2;
  } else {
    median = sortedData[_size / 2];
  }

  _hasCache = true;
  _cache = median;
  delete[] sortedData;
  return median;
}