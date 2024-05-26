#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

#include <Arduino.h>

class MedianFilter {
 public:
  MedianFilter(uint8_t size);
  void addValue(uint32_t value);
  uint32_t getMedian();

 private:
  uint8_t _maxSize;
  uint8_t _size = 0;
  uint8_t _index = 0;
  uint32_t* _rawData;
  bool _hasCache = false;
  uint32_t _cache;
};

#endif