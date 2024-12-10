#include "Encoder.h"

Encoder::Encoder(HardwareSerial* serial, uint8_t renPin, uint8_t dePin) {
  this->ren = renPin;
  this->de = dePin;
  this->pSerial = serial;
}

Encoder::~Encoder() {}

uint32_t Encoder::requestData() {
  digitalWrite(de, HIGH);
  digitalWrite(ren, HIGH);
  // pSerial->print(DataID3);
  pSerial->write((uint8_t)DataID0);
  pSerial->flush();
  digitalWrite(de, LOW);
  digitalWrite(ren, LOW);

  return 0;

  // delayMicroseconds(50);
  // const int byteCount = 6;
  // // while (pSerial->available() < byteCount)
  // //   ;
  // while (pSerial->available() >= byteCount) {
  //   uint8_t data[byteCount] = {0};
  //   uint32_t as = 0;
  //   pSerial->readBytes(data, byteCount);
  //   as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
  //        ((uint32_t)(data[4]) << 16);

  //   return as;
  // }
}