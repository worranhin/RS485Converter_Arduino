#include "hal_conf_extra.h"
#include "config.h"
#include <Arduino.h>
// #include <STM32FreeRTOS.h>
#include "Encoder.h"
#include "MedianFilter.h"

uint8_t encoderData1[8];
uint8_t encoderData2[8];
uint8_t encoderData3[8];
bool waitingEncoder1 = false;
bool waitingEncoder2 = false;
bool waitingEncoder3 = false;

// HardwareSerial EncoderSerial1(ENCODER_RX1, ENCODER_TX1);
// HardwareSerial EncoderSerial2(ENCODER_RX2, ENCODER_TX2);
// HardwareSerial EncoderSerial3(ENCODER_RX3, ENCODER_TX3);
// HardwareSerial MasterSerial(MASTER485_RX, MASTER485_TX);

Encoder Encoder1(&Serial1, ENCODER_REN1, ENCODER_DE1);
Encoder Encoder2(&Serial2, ENCODER_REN2, ENCODER_DE2);
Encoder Encoder3(&Serial3, ENCODER_REN3, ENCODER_DE3);
MedianFilter filter1(5);
MedianFilter filter2(5);
MedianFilter filter3(5);
// Encoder Encoder1(&EncoderSerial1, ENCODER_REN1, ENCODER_DE1);
// Encoder Encoder2(&EncoderSerial2, ENCODER_REN2, ENCODER_DE2);
// Encoder Encoder3(&EncoderSerial3, ENCODER_REN3, ENCODER_DE3);

void TaskReadEncoder(void* pvParameters);
void TaskResponseMaster(void* pvParameters);
void TaskBlink(void* pvParameters);
void RequestEncoderLoop();
void ResponseEncoderLoop();
void ResponseMasterLoop();

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

  Serial1.begin(2500000);
  Serial2.begin(2500000);
  Serial3.begin(2500000);
  Serial6.begin(115200);
  // EncoderSerial1.begin(2500000);
  // EncoderSerial2.begin(2500000);
  // EncoderSerial3.begin(2500000);
  // MasterSerial.begin(115200);

  while (!Serial1 || !Serial2 || !Serial3 || !Serial6)
    ;
  // while (!EncoderSerial1 || !EncoderSerial2 || !EncoderSerial3 ||
  // !MasterSerial)
  //   ;

  // xTaskCreate(TaskReadEncoder, "TaskReadEncoder", 128, NULL, 1, NULL);
  // xTaskCreate(TaskResponseMaster, "TaskResponseMaster", 128, NULL, 2, NULL);
  // xTaskCreate(TaskBlink, "TaskBlink", 128, NULL, 1, NULL);

  // vTaskStartScheduler();
  // Serial6.println("Error: Insufficient RAM");
}

void loop() {
  RequestEncoderLoop();
  ResponseEncoderLoop();
  ResponseMasterLoop();
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

void RequestEncoderLoop() {
  if (!waitingEncoder1) {
    Encoder1.requestData();
    waitingEncoder1 = true;
  }
  if (!waitingEncoder2) {
    Encoder2.requestData();
    waitingEncoder2 = true;
  }
  if (!waitingEncoder3) {
    Encoder3.requestData();
    waitingEncoder3 = true;
  }
}

void ResponseEncoderLoop() {
  if (Encoder1.pSerial->available() >= 8) {
    uint8_t data[8];
    uint32_t as = 0;
    Encoder1.pSerial->readBytes(data, 8);
    as = ((uint32_t)(data[0])) | ((uint32_t)(data[1]) << 8) |
         ((uint32_t)(data[2]) << 16);
    filter1.addValue(as);
    waitingEncoder1 = false;
  }

  if (Encoder2.pSerial->available() >= 8) {
    uint8_t data[8];
    uint32_t as = 0;
    Encoder2.pSerial->readBytes(data, 8);
    as = ((uint32_t)(data[0])) | ((uint32_t)(data[1]) << 8) |
         ((uint32_t)(data[2]) << 16);
    filter2.addValue(as);
    waitingEncoder2 = false;
  }

  if (Encoder3.pSerial->available() >= 8) {
    uint8_t data[8];
    uint32_t as = 0;
    Encoder3.pSerial->readBytes(data, 8);
    as = ((uint32_t)(data[0])) | ((uint32_t)(data[1]) << 8) |
         ((uint32_t)(data[2]) << 16);
    filter3.addValue(as);
    waitingEncoder3 = false;
  }
}

void ResponseMasterLoop() {
  if (Serial6.available()) {
    uint8_t code = 0;
    Serial6.readBytes(&code, 1);
    if (code == 1) {
      uint32_t value1 = filter1.getMedian();
      uint32_t value2 = filter2.getMedian();
      uint32_t value3 = filter3.getMedian();

      uint8_t data1[3] = {(uint8_t)(value1), (uint8_t)(value1 >> 8),
                          (uint8_t)(value1 >> 16)};
      uint8_t data2[3] = {(uint8_t)(value2), (uint8_t)(value2 >> 8),
                          (uint8_t)(value2 >> 16)};
      uint8_t data3[3] = {(uint8_t)(value3), (uint8_t)(value3 >> 8),
                          (uint8_t)(value3 >> 16)};

      Serial6.write(data1, 3);
      Serial6.write(data2, 3);
      Serial6.write(data3, 3);
    }
  }
}

// void TaskReadEncoder(void* pvParameters) {
//   for (;;) {
//     Encoder1.requestData();
//     Encoder2.requestData();
//     Encoder3.requestData();

//     while (Encoder1.pSerial->available() < 8)
//       ;
//     Encoder1.pSerial->readBytes(encoderData1, 8);

//     while (Encoder2.pSerial->available() < 8)
//       ;
//     Encoder2.pSerial->readBytes(encoderData2, 8);

//     while (Encoder3.pSerial->available() < 8)
//       ;
//     Encoder3.pSerial->readBytes(encoderData3, 8);
//   }
// }

// void TaskResponseMaster(void* pvParameters) {
//   for (;;) {
//     if (Serial6.available()) {
//       uint8_t code = 0;
//       Serial6.readBytes(&code, 1);
//       if (code == 1) {
//         Serial6.write(encoderData1, 8);
//         Serial6.write(encoderData2, 8);
//         Serial6.write(encoderData3, 8);
//       }
//     }
//   }
// }

// void TaskBlink(void* pvParameters) {
//   for (;;) {
//     digitalWrite(BUILTIN_LED, HIGH);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//     digitalWrite(BUILTIN_LED, LOW);
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
//   }
// }