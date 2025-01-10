#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "Crc.h"

// const uint8_t DataID0 = 0b00000010;

enum DataID: uint8_t {
  DataID0 = 0b00000010,
  DataID1 = 0b10001010,
  DataID2 = 0b10010010,
  DataID3 = 0b00011010
};

enum EncoderError {
  OK,
  WrongControlField,
  WrongCrcField,
  WrongStatusField
};

class Encoder {
public:
  EncoderError error = OK;
 private:
  CrcHelper crcHelper;
  /* data */
 public:
  Encoder(HardwareSerial* serial, uint8_t renPin, uint8_t dePin);
  ~Encoder();
  uint32_t requestData();
  uint32_t readData();

  uint8_t ren;
  uint8_t de;
  HardwareSerial* pSerial;
};

#endif //ENCODER_H