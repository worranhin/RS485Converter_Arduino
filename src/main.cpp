#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "Encoder.h"
#include "config.h"

uint8_t encoderData1[8];
uint8_t encoderData2[8];
uint8_t encoderData3[8];

HardwareSerial EncoderSerial1(ENCODER_RX1, ENCODER_TX1);
HardwareSerial EncoderSerial2(ENCODER_RX2, ENCODER_TX2);
HardwareSerial EncoderSerial3(ENCODER_RX3, ENCODER_TX3);
HardwareSerial MasterSerial(MASTER485_RX, MASTER485_TX);

Encoder Encoder1(&EncoderSerial1, ENCODER_REN1, ENCODER_DE1);
Encoder Encoder2(&EncoderSerial2, ENCODER_REN2, ENCODER_DE2);
Encoder Encoder3(&EncoderSerial3, ENCODER_REN3, ENCODER_DE3);

void TaskReadEncoder(void* pvParameters);
void TaskResponseMaster(void* pvParameters);
void TaskBlink(void* pvParameters);

void setup() {
  // put your setup code here, to run once:
  pinMode(BUILTIN_LED, OUTPUT);

  pinMode(ENCODER_REN1, OUTPUT);
  pinMode(ENCODER_DE1, OUTPUT);
  pinMode(ENCODER_REN2, OUTPUT);
  pinMode(ENCODER_DE2, OUTPUT);
  pinMode(ENCODER_REN3, OUTPUT);
  pinMode(ENCODER_DE3, OUTPUT);

  digitalWrite(ENCODER_REN1, LOW);
  digitalWrite(ENCODER_DE1, LOW);
  digitalWrite(ENCODER_REN2, LOW);
  digitalWrite(ENCODER_DE2, LOW);
  digitalWrite(ENCODER_REN3, LOW);
  digitalWrite(ENCODER_DE3, LOW);

  EncoderSerial1.begin(2500000);
  EncoderSerial2.begin(2500000);
  EncoderSerial3.begin(2500000);
  MasterSerial.begin(115200);

  while (!EncoderSerial1 || !EncoderSerial2 || !EncoderSerial3 || !MasterSerial)
    ;

  xTaskCreate(TaskReadEncoder, "TaskReadEncoder", 128, NULL, 1, NULL);
  xTaskCreate(TaskResponseMaster, "TaskResponseMaster", 128, NULL, 2, NULL);
  xTaskCreate(TaskBlink, "TaskBlink", 128, NULL, 1, NULL);

  vTaskStartScheduler();
  MasterSerial.println("Error: Insufficient RAM");
}

void loop() {
  // digitalWrite(BUILTIN_LED, HIGH);
  // delay(1000);
  // digitalWrite(BUILTIN_LED, LOW);
  // delay(1000);

  // digitalWrite(MASTER485_REN, HIGH);
  // digitalWrite(MASTER485_DE, HIGH);
  // MasterSerial.println("Hello World");
  // digitalWrite(MASTER485_REN, LOW);
  // digitalWrite(MASTER485_DE, LOW);
  // delay(1000);
}

void TaskReadEncoder(void* pvParameters) {
  for (;;) {
    Encoder1.requestData();
    Encoder2.requestData();
    Encoder3.requestData();

    while (Encoder1.pSerial->available() < 8)
      ;
    Encoder1.pSerial->readBytes(encoderData1, 8);

    while (Encoder2.pSerial->available() < 8)
      ;
    Encoder2.pSerial->readBytes(encoderData2, 8);

    while (Encoder3.pSerial->available() < 8)
      ;
    Encoder3.pSerial->readBytes(encoderData3, 8);
  }
}

void TaskResponseMaster(void* pvParameters) {
  for (;;) {
    if (MasterSerial.available()) {
      uint8_t code = 0;
      MasterSerial.readBytes(&code, 1);
      if (code == 1) {
        MasterSerial.write(encoderData1, 8);
        MasterSerial.write(encoderData2, 8);
        MasterSerial.write(encoderData3, 8);
      }
    }
  }
}

void TaskBlink(void* pvParameters) {
  for (;;) {
    digitalWrite(BUILTIN_LED, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(BUILTIN_LED, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}