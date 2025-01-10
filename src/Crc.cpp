#include "Crc.h"

CrcHelper::CrcHelper() {
  uint16_t i, j;
  uint8_t crcResult;
  for (j = 0; j < 256; j++) {
    crcResult = j;
    for (i = 0; i < 8; i++) {
      if (crcResult & 0x80) {
        crcResult = (crcResult << 1) ^ 0x01;
      } else {
        crcResult <<= 1;
      }
    }
    crc_8x1[j] = (crcResult & 0x00ff);
  }
}

CrcHelper::~CrcHelper() {}

uint8_t CrcHelper::Check(uint8_t* crcBuf, uint8_t length) {
  uint8_t crcResult = 0;
  uint8_t crcLength = 0;
  while (crcLength < length) {
    crcResult ^= crcBuf[crcLength];
    crcResult = (crcResult & 0x00ff);
    crcLength++;
    crcResult = crc_8x1[crcResult];
  }
  return crcResult;
}
