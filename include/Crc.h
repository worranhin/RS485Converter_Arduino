#include <Arduino.h>

class CrcHelper {
  private:
    uint8_t crc_8x1[256];
  public:
    CrcHelper();
    ~CrcHelper();
    uint8_t Check(uint8_t* crcBuf, uint8_t length);
};