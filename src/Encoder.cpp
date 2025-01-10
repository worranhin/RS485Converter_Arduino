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

/**
 * @brief read data from the encoder
 * @return the data read from the encoder, if the data is invalid, return 0xffffffff
 * @details
 * This function will send the command to request the encoder to send data back,
 * then read the data from the encoder and return the value.
 * If the data received is not valid (i.e. the first byte is not DataID0), this
 * function will return 0xffffffff.
 */
uint32_t Encoder::readData() {
  const int bytesToRead = 6;
  uint8_t rxBuffer[bytesToRead] = {0};
  uint32_t value = 0;

  digitalWrite(de, HIGH);
  digitalWrite(ren, HIGH);
  pSerial->write((uint8_t)DataID0);
  pSerial->flush();
  digitalWrite(de, LOW);
  digitalWrite(ren, LOW);


  pSerial->readBytes(rxBuffer, bytesToRead);

  // 验证数据
  // if (rxBuffer[0] != DataID0) {  // 这个头字节，产品返回的值很奇怪
  //   error = WrongControlField;
  //   return 0xffffffff;
  // }

  // uint8_t crcField = rxBuffer[5];
  // if (crcField != crcHelper.Check(&rxBuffer[0], bytesToRead - 1)) {
  //   error = WrongCrcField;
  //   return 0xffffffff;
  // }

  // uint8_t statusField = rxBuffer[1];
  // bool err1 = bitRead(statusField, 4);
  // bool err2 = bitRead(statusField, 5);
  // if (err1 || err2) {
  //   error = WrongStatusField;
  //   return 0xffffffff;
  // }

  // 解析数据
  value = ((uint32_t)(rxBuffer[2])) | ((uint32_t)(rxBuffer[3]) << 8) |
       ((uint32_t)(rxBuffer[4]) << 16);

  return value;
}