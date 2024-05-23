#include "Encoder.h"

Encoder::Encoder(HardwareSerial* serial, uint8_t renPin, uint8_t dePin) {
  this->ren = renPin;
  this->de = dePin;
  this->pSerial = serial;
}

Encoder::~Encoder() {}

void Encoder::requestData() {
  digitalWrite(de, HIGH);
  digitalWrite(ren, HIGH);
  // pSerial->print(DataID3);
  pSerial->write(DataID3);
  digitalWrite(de, LOW);
  digitalWrite(ren, LOW);
}