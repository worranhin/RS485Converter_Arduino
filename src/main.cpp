#include "Encoder.h"
#include "MedianFilter.h"
#include "config.h"
#include "hal_conf_extra.h"
#include <Arduino.h>

// #define DEBUG_SAMPLE_FREQ
#define DEBUG_ONE_ENCODER
#define DEBUG_SEND_DATA

#define MasterSerial Serial6

// uint8_t encoderData1[8];
// uint8_t encoderData2[8];
// uint8_t encoderData3[8];
static bool waitingEncoder1 = false;
static bool waitingEncoder2 = false;
static bool waitingEncoder3 = false;

// 测试采样频率
#ifdef DEBUG_SAMPLE_FREQ
uint32_t lastSampleTime;
uint32_t beginSampleTime = 0;
uint32_t endSampleTime = 0;
#endif

Encoder Encoder1(&Serial1, ENCODER_REN1, ENCODER_DE1);
#ifndef DEBUG_ONE_ENCODER
Encoder Encoder2(&Serial2, ENCODER_REN2, ENCODER_DE2);
Encoder Encoder3(&Serial3, ENCODER_REN3, ENCODER_DE3);
#endif

MedianFilter filter1(5);
#ifndef DEBUG_ONE_ENCODER
MedianFilter filter2(5);
MedianFilter filter3(5);
#endif

void RequestEncoderLoop();
void ResponseEncoderLoop();
void ResponseMasterLoop();
void RequestEncoderLoop_block();
void BlinkLoop();
// void enableTX(int renPin, int dePin);
// void enableRX(int renPin, int dePin);

/**
 * @brief System Clock Configuration
 * @retval None
 */
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void TestResponseMasterLoop() {
  while (Serial6.available()) {
    uint8_t code = 0;
    Serial6.readBytes(&code, 1);

    digitalWrite(ENCODER_DE1, HIGH);
    digitalWrite(ENCODER_REN1, HIGH);
    Serial1.write(code);
    Serial1.flush();
    digitalWrite(ENCODER_DE1, LOW);
    digitalWrite(ENCODER_REN1, LOW);
    delay(10);
    while (Serial1.available()) {
      byte incomingByte = Serial1.read();
      digitalWrite(MASTER485_DE, HIGH);
      digitalWrite(MASTER485_REN, HIGH);
      Serial6.write(incomingByte);
      Serial6.flush();
      digitalWrite(MASTER485_DE, LOW);
      digitalWrite(MASTER485_REN, LOW);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);

  pinMode(ENCODER_REN1, OUTPUT);
  pinMode(ENCODER_DE1, OUTPUT);

#ifndef DEBUG_ONE_ENCODER
  pinMode(ENCODER_REN2, OUTPUT);
  pinMode(ENCODER_DE2, OUTPUT);
  pinMode(ENCODER_REN3, OUTPUT);
  pinMode(ENCODER_DE3, OUTPUT);
#endif

  pinMode(MASTER485_REN, OUTPUT);
  pinMode(MASTER485_DE, OUTPUT);

  digitalWrite(ENCODER_REN1, LOW);
  digitalWrite(ENCODER_DE1, LOW);

#ifndef DEBUG_ONE_ENCODER
  digitalWrite(ENCODER_REN2, LOW);
  digitalWrite(ENCODER_DE2, LOW);
  digitalWrite(ENCODER_REN3, LOW);
  digitalWrite(ENCODER_DE3, LOW);
#endif

  digitalWrite(MASTER485_REN, LOW);
  digitalWrite(MASTER485_DE, LOW);

  Serial1.begin(2500000);

#ifndef DEBUG_ONE_ENCODER
  Serial2.begin(2500000);
  Serial3.begin(2500000);
#endif
  Serial6.begin(115200);

  waitingEncoder1 = false;

#ifndef DEBUG_ONE_ENCODER
  waitingEncoder2 = false;
  waitingEncoder3 = false;
#endif

#ifndef DEBUG_ONE_ENCODER
  while (!Serial1 || !Serial2 || !Serial3 || !Serial6)
    ;
#else
  while (!Serial1 || !Serial6)
    ;
#endif
}

void loop() {
  RequestEncoderLoop_block();
  ResponseMasterLoop();
  BlinkLoop();
  // TestResponseMasterLoop();
  // RequestEncoderLoop();
  // ResponseEncoderLoop();
}

void RequestEncoderLoop_block() {
  Encoder1.requestData();
#ifndef DEBUG_ONE_ENCODER
  Encoder2.requestData();
  Encoder3.requestData();
#endif

  // delayMicroseconds(20);

  const int byteCount = 6;
  uint8_t data[byteCount] = {0};
  uint32_t as = 0;
  // while (Encoder1.pSerial->available() >= byteCount) {
  Encoder1.pSerial->readBytes(data, byteCount);

#ifdef DEBUG_SEND_DATA
  digitalWrite(MASTER485_REN, HIGH);
  digitalWrite(MASTER485_DE, HIGH);
  Serial6.write(data, byteCount);
  Serial6.flush();
  digitalWrite(MASTER485_REN, LOW);
  digitalWrite(MASTER485_DE, LOW);
  delay(1000);
#endif

  as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
       ((uint32_t)(data[4]) << 16);
  filter1.addValue(as);
  // }

#ifndef DEBUG_ONE_ENCODER
  // while (Encoder2.pSerial->available() >= byteCount) {
  // uint8_t data[byteCount] = {0};
  // uint32_t as = 0;
  Encoder2.pSerial->readBytes(data, byteCount);
  as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
       ((uint32_t)(data[4]) << 16);
  filter2.addValue(as);
  // }

  // while (Encoder3.pSerial->available() >= byteCount) {
  // uint8_t data[byteCount] = {0};
  // uint32_t as = 0;
  Encoder3.pSerial->readBytes(data, byteCount);
  as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
       ((uint32_t)(data[4]) << 16);
  filter3.addValue(as);
// }
#endif

  // DEBUG

  // uint32_t value1 = filter1.getMedian();
  // uint32_t value2 = filter2.getMedian();
  // uint32_t value3 = filter3.getMedian();

  // // uint8_t data1[3] = {(uint8_t)(value1), (uint8_t)(value1 >> 8),
  // //                     (uint8_t)(value1 >> 16)};
  // // uint8_t data2[3] = {(uint8_t)(value2), (uint8_t)(value2 >> 8),
  // //                     (uint8_t)(value2 >> 16)};
  // // uint8_t data3[3] = {(uint8_t)(value3), (uint8_t)(value3 >> 8),
  // //                     (uint8_t)(value3 >> 16)};

  // digitalWrite(MASTER485_REN, HIGH);
  // digitalWrite(MASTER485_DE, HIGH);
  // // Serial6.write(data1, 3);
  // // Serial6.write(data2, 3);
  // // Serial6.write(data3, 3);
  // // double angle1 = (double)value1 / (double)pow(2, 19) * 360.0;
  // // double angle2 = (double)value2 / (double)pow(2, 19) * 360.0;
  // // double angle3 = (double)value3 / (double)pow(2, 19) * 360.0;
  // // Serial6.printf("%f, %f, %f\n", angle1, angle2, angle3);
  // Serial6.printf("%d, %d, %d\n", value1, value2, value3);

  // Serial6.flush();
  // digitalWrite(MASTER485_REN, LOW);
  // digitalWrite(MASTER485_DE, LOW);

  // DEBUG END

#ifdef DEBUG_SAMPLE_FREQ
  endSampleTime = micros();
  uint32_t samplePeriod = endSampleTime - beginSampleTime;
  digitalWrite(MASTER485_DE, HIGH);
  digitalWrite(MASTER485_REN, HIGH);
  MasterSerial.print(beginSampleTime);
  MasterSerial.print(", ");
  MasterSerial.println(samplePeriod);
  MasterSerial.flush();
  digitalWrite(MASTER485_DE, LOW);
  digitalWrite(MASTER485_REN, LOW);
  beginSampleTime = micros();
#endif
}

void RequestEncoderLoop() {
  if (waitingEncoder1 == false && waitingEncoder2 == false &&
      waitingEncoder3 == false) {
    Encoder1.requestData();
#ifndef DEBUG_ONE_ENCODER
    Encoder2.requestData();
    Encoder3.requestData();
#endif
    waitingEncoder1 = true;
#ifndef DEBUG_ONE_ENCODER
    waitingEncoder2 = true;
    waitingEncoder3 = true;
#endif
  }
}

void ResponseEncoderLoop() {
  const int byteCount = 6;
  if (Encoder1.pSerial->available() >= byteCount) {
    uint8_t data[byteCount] = {0};
    uint32_t as = 0;
    Encoder1.pSerial->readBytes(data, byteCount);
    as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
         ((uint32_t)(data[4]) << 16);
    filter1.addValue(as);
    waitingEncoder1 = false;
  }

#ifndef DEBUG_ONE_ENCODER
  if (Encoder2.pSerial->available() >= byteCount) {
    uint8_t data[byteCount];
    uint32_t as = 0;
    Encoder2.pSerial->readBytes(data, byteCount);
    as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
         ((uint32_t)(data[4]) << 16);
    filter2.addValue(as);
    waitingEncoder2 = false;
  }

  if (Encoder3.pSerial->available() >= byteCount) {
    uint8_t data[byteCount];
    uint32_t as = 0;
    Encoder3.pSerial->readBytes(data, byteCount);
    as = ((uint32_t)(data[2])) | ((uint32_t)(data[3]) << 8) |
         ((uint32_t)(data[4]) << 16);
    filter3.addValue(as);
    waitingEncoder3 = false;
  }
#endif
}

void ResponseMasterLoop() {
  if (Serial6.available()) {
    uint8_t code = 0;
    Serial6.readBytes(&code, 1);

    if (code == 0x01) {
      uint32_t value1 = filter1.getMedian();
#ifndef DEBUG_ONE_ENCODER
      uint32_t value2 = filter2.getMedian();
      uint32_t value3 = filter3.getMedian();
#endif

      uint8_t data1[3] = {(uint8_t)(value1), (uint8_t)(value1 >> 8),
                          (uint8_t)(value1 >> 16)};
#ifndef DEBUG_ONE_ENCODER
      uint8_t data2[3] = {(uint8_t)(value2), (uint8_t)(value2 >> 8),
                          (uint8_t)(value2 >> 16)};
      uint8_t data3[3] = {(uint8_t)(value3), (uint8_t)(value3 >> 8),
                          (uint8_t)(value3 >> 16)};
#endif

      digitalWrite(MASTER485_REN, HIGH);
      digitalWrite(MASTER485_DE, HIGH);
      Serial6.write(data1, 3);
#ifndef DEBUG_ONE_ENCODER
      Serial6.write(data2, 3);
      Serial6.write(data3, 3);
#endif
      Serial6.flush();
      digitalWrite(MASTER485_REN, LOW);
      digitalWrite(MASTER485_DE, LOW);
    }
  }
}

void BlinkLoop() {
  static uint32_t lastTime = 0;
  if (millis() - lastTime > 1000) {
    digitalToggle(BUILTIN_LED);
    lastTime = millis();
  }
}