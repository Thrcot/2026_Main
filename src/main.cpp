#include <Arduino.h>
#include <Wire.h>

// define pins

// Kicker
#define Kick PA1
#define ADCCatch PA4

// Buttons
#define Back PA5
#define Pause PA6
#define Enter PA7
#define Next PB4
#define Option PB5

// Debug
#define UART2_TX PA2
#define UART2_RX PA3

// Line Sensor
#define UART3_TX PC10
#define UART3_RX PC11

#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define I2C2_SCL PB10
#define I2C2_SDA PB3

// Ball Sensor
#define A1 PC0
#define A2 PC2
#define A3 PC1
#define A4 PC3
#define A5 PC4
#define A6 PB0
#define A7 PC5
#define A8 PB1

#define LED1 PC14
#define LED2 PA0
#define LED3 PC15
#define LED4 PC13
#define LED5 PB15
#define LED6 PB12
#define LED7 PB14
#define LED8 PB13

// PWM
#define TIM3_CH1 PC6
#define TIM3_CH2 PC7
#define TIM3_CH3 PC8
#define TIM3_CH4 PC9
#define TIM1_CH1 PA8
#define TIM1_CH2 PA9
#define TIM1_CH3 PA10
#define TIM1_CH4 PA11
#define TIM4_CH1 PB6
#define TIM4_CH2 PB7

// define objects
HardwareSerial SerialPC(UART2_TX, UART2_RX);
HardwareSerial SerialLine(UART3_TX, UART3_RX);
TwoWire Wire1(I2C1_SCL, I2C1_SDA);
TwoWire Wire2(I2C2_SCL, I2C2_SDA);

void setup() {
  SerialPC.begin(115200);
  SerialLine.begin(115200);
  Wire1.begin();
  Wire2.begin();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);

  pinMode(TIM3_CH1, OUTPUT);
  pinMode(TIM3_CH2, OUTPUT);
  pinMode(TIM3_CH3, OUTPUT);
  pinMode(TIM3_CH4, OUTPUT);
  pinMode(TIM1_CH1, OUTPUT);
  pinMode(TIM1_CH2, OUTPUT);
  pinMode(TIM1_CH3, OUTPUT);
  pinMode(TIM1_CH4, OUTPUT);
  pinMode(TIM4_CH1, OUTPUT);
  pinMode(TIM4_CH2, OUTPUT);

  pinMode(Kick, OUTPUT);
  pinMode(ADCCatch, INPUT);

  pinMode(Back, INPUT);
  pinMode(Pause, INPUT);
  pinMode(Enter, INPUT);
  pinMode(Next, INPUT);
  pinMode(Option, INPUT);
}

void loop() {

}