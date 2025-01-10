#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin Define

#define ENCODER_RX1 PB7
#define ENCODER_TX1 PB6
#define ENCODER_REN1 PD0
#define ENCODER_DE1 PD1

#define ENCODER_RX2 PA3
#define ENCODER_TX2 PA2
#define ENCODER_REN2 PD2
#define ENCODER_DE2 PD3

#define ENCODER_RX3 PB11
#define ENCODER_TX3 PB10
#define ENCODER_REN3 PD4
#define ENCODER_DE3 PD5

#define MASTER485_RX PC7
#define MASTER485_TX PC6
#define MASTER485_REN PD6
#define MASTER485_DE PD7

#define BUILTIN_LED PA1

#define LED_ON HIGH
#define LED_OFF LOW

#endif