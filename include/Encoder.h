#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// const uint8_t DataID0 = 0b00000010;

enum DataID: uint8_t {
  DataID0 = 0b00000010,
  DataID1 = 0b10001010,
  DataID2 = 0b10010010,
  DataID3 = 0b00011010
};

class Encoder {
 private:
  /* data */
 public:
  Encoder(HardwareSerial* serial, uint8_t renPin, uint8_t dePin);
  ~Encoder();
  void requestData();

  uint8_t ren;
  uint8_t de;
  HardwareSerial* pSerial;
};



#endif