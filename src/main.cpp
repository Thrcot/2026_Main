#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "display.h"
#include "stm32f4xx_hal.h"

#include <math.h>

// define pins

//  Kicker
#define Kick PA1
#define ADCCatch PA4

//  Buttons
#define Back PA5
#define Pause PA6
#define Enter PA7
#define Next PB4
#define Option PB5

//  Debug
#define UART2_TX PA2
#define UART2_RX PA3

//  Line Sensor
#define UART3_TX PC10
#define UART3_RX PC11

#define I2C1_SCL PB8
#define I2C1_SDA PB9

#define I2C2_SCL PB10
#define I2C2_SDA PB3

//  Ball Sensor
#define A1 PC0
#define A2 PC2
#define A3 PC1
#define A4 PC3
#define A5 PC4
#define A6 PB0
#define A7 PC5
#define A8 PB1

struct Ball {
  double Angle;
  double Distance;
};

const int BALL[8] = {A1, A2, A3, A4, A5, A6, A7, A8};
double BALLANGLE[8] = {0, 45, 90, 135, 180, 225, 270, 315};

#define LED1 PC14
#define LED2 PA0
#define LED3 PC15
#define LED4 PC13
#define LED5 PB15
#define LED6 PB12
#define LED7 PB14
#define LED8 PB13

const int LED[8] = {LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8};

//  PWM
// FL
#define FL_FWD PC7
#define FL_REV PC6
// BL
#define BL_FWD PC9
#define BL_REV PC8
// BR
#define BR_FWD PA9
#define BR_REV PA8
// FR
#define FR_FWD PA11
#define FR_REV PA10
//   Dribble
#define TIM4_CH1 PB6
#define TIM4_CH2 PB7

//  define objects
HardwareSerial SerialPC(UART2_RX, UART2_TX);
HardwareSerial SerialLine(UART3_RX, UART3_TX);
TwoWire Wire1(I2C1_SDA, I2C1_SCL);
TwoWire Wire2(I2C2_SDA, I2C2_SCL);

#define SCREEN_ADDR 0x3C  //wire1
#define BNO055_ADDR 0x28  //wire2
#define EEPROM_ADDR 0x50  //wire2

#define EE_LINE_THRESHOLD 0x0000  //EEPROMアドレス

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire2);

#define LINE_SetThreshold 0xAB
#define LINE_SENSOR_INFO 0xAC
#define LINE_SENSOR_HEADER 0xAD
#define LINE_CARIBRATION_ERROR 0xAE
#define LINE_ANGLE_INFO 0xAF

// DMA Setting
#define SENSOR_CH 8
#define SAMPLE_NUM 32

// HALハンドラ
ADC_HandleTypeDef hadc1; //DMA
ADC_HandleTypeDef hadc2; //Kicker
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

HardwareTimer *tim6 = new HardwareTimer(TIM6);

// DMA Buffer
uint16_t adc_buf[SENSOR_CH * SAMPLE_NUM];
volatile uint16_t sensor_avg[SENSOR_CH];
volatile bool adc_ready = false;

// MARK: constants
bool gameFlag = false;
uint8_t line_threshold = 155; //デフォルト(仮)

int PWM_limit = 230; // yukuyukuMD MAX is 230.

int basespeed = 80; //0~255
double Kp = 0.44;
double Ki = 0.0;
double Kd = 0.03;
double preTime = 0.0;
double preHeading = 0.0;
double P = 0.0, I = 0.0, D = 0.0, preP = 0.0;

double headingOffset = 0.0;
double prevTime = 0.0;
// MARK: function
void TIM2_Init(void);
void ADC1_DMA_Init(void);
void ADC2_Init(void);
void processADC();
uint16_t readCatch();
double wrapAngle180(double angle);
void writeEEPROM(int addr, byte data);
byte readEEPROM(int addr);
Ball getBall();
double getBallAngle();
double getHeading();
double getGyroZ();
void getIMU(double *heading, double *gyroZ);
int16_t getLineAngle();
void motor_test();
void setMotor(int pwm, int MDpin1, int MDpin2);
void move_motor(int speed, double target_angle, double heading, double gyroZ, double tarHeading);
void kick();
void lcd_drawarrow(double angle);
void lcd_drawLineSensors(bool lineSensor[19]);
void lcd_menu();
bool getLineSensorValues(bool lineSensor[19]);
void lineCalibration();
void resetHeadingZero();
void setLineThreshold(uint8_t threshold);
void saveLineThreshold(uint8_t threshold);
uint8_t loadLineThreshold();

// HAL Callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  adc_ready = true;
}

extern "C" void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

// MARK: setup
void setup() {
  SerialPC.begin(115200);
  SerialLine.begin(115200);
  Wire1.begin();
  Wire2.begin();
  delay(1000);
  SerialPC.println("[Debug] Setup start");

  for (byte addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      SerialPC.print("[Wire1] Found I2C device at 0x");
      SerialPC.println(addr, HEX);
    }
  }

  for (byte addr = 1; addr < 127; addr++) {
    Wire2.beginTransmission(addr);
    if (Wire2.endTransmission() == 0) {
      SerialPC.print("[Wire2] Found I2C device at 0x");
      SerialPC.println(addr, HEX);
    }
  }

  SerialPC.println("[Debug] Display initialize");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
	  Serial.println("SSD1306 initialization failed");
	  return;
	}

  display.clearDisplay();
  display.display();
  display.setTextColor(WHITE);
	display.setCursor(0, 0);
  display.drawBitmap(0, 0, THRCOT_BMP, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.display();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);

  for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], LOW);
  }

  pinMode(FL_FWD, OUTPUT);
  pinMode(FL_REV, OUTPUT);
  pinMode(BL_FWD, OUTPUT);
  pinMode(BL_REV, OUTPUT);
  pinMode(BR_FWD, OUTPUT);
  pinMode(BR_REV, OUTPUT);
  pinMode(FR_FWD, OUTPUT);
  pinMode(FR_REV, OUTPUT);
  pinMode(TIM4_CH1, OUTPUT);
  pinMode(TIM4_CH2, OUTPUT);

  pinMode(Kick, OUTPUT);
  digitalWrite(Kick, LOW);

  pinMode(Back, INPUT);
  pinMode(Pause, INPUT);
  pinMode(Enter, INPUT);
  pinMode(Next, INPUT);
  pinMode(Option, INPUT);

  // bno setup
  SerialPC.println("[Debug] BNO initialize");
  while (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], !digitalRead(LED[i]));
  }
  delay(100);
  }
  for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], LOW);
  }
  bno.setExtCrystalUse(true);
  //bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(500);
  resetHeadingZero();

  SerialPC.println("[Debug] Line initialize");
  // line sensor setup
  line_threshold = loadLineThreshold();
  setLineThreshold(line_threshold);      // ラインセンサ側に送信

  // ADC DMA Init
  SerialPC.println("[Debug] DMA ADC initialize");
  ADC1_DMA_Init();
  TIM2_Init();
  HAL_ADC_Start_DMA(&hadc1,
                    (uint32_t*)adc_buf,
                    SENSOR_CH * SAMPLE_NUM);

  SerialPC.println("[Debug] Kicker initialize");
  ADC2_Init();

  delay(1000);
  display.clearDisplay();
  display.display();

  SerialPC.println("[Debug] Setup end");
}


// MARK: main loop
void loop() {
  if (adc_ready) {
    adc_ready = false;
    processADC();
  }
  if(gameFlag == true){
    SerialPC.println("[Debug] Game loop");
    static int16_t lastLineAngle = -1;
    static unsigned long lastLineTime = 0;
    static int lineAngle = -1;
    static unsigned long lastHeadingTime = 0;
    static int targetHeading = 0;
    double speed = basespeed;

    Ball b = getBall();
    double targetAngle = b.Angle;

    static double prevBallErr = 0;
    double ballErr = b.Angle;
    double dBallErr = ballErr - prevBallErr;
    prevBallErr = ballErr;

    double KP_ball = 0.2;
    double KD_ball = 0.0;

    double pd = KP_ball * ballErr + KD_ball * dBallErr;

    if (b.Distance >= 210) {
      double rad = b.Angle * PI / 180.0;

      targetAngle = b.Angle + 50 * sin(rad) + pd;

      speed = basespeed * (0.7 + 0.3 * abs(cos(rad)));

    } else if (b.Distance >= 150) {
      speed = basespeed;
    } else {
      speed = 0;
    }

    display.clearDisplay();
    lcd_drawarrow(targetAngle);
    display.display();

    lineAngle = getLineAngle();   //ライン踏んだ時の移動角
    if(lineAngle != -1){
      lastLineAngle = lineAngle;
      lastLineTime = millis();
      if(lineAngle > 45 && lineAngle < 135){
        targetHeading = 45;
        lastHeadingTime = millis();
      }else if(lineAngle > 225 && lineAngle < 315){
        targetHeading = -45;
        lastHeadingTime = millis();
      }
    }

    if(lastLineAngle != -1 && (millis() - lastLineTime) < 50){  // 50msは後退する
      targetAngle = wrapAngle180((double)lastLineAngle);
    }
    if(targetHeading != 0 && (millis() - lastHeadingTime) > 500){ // ライン踏んでから0.5秒後には目標角度をリセット
      targetHeading = 0;
    }

    double heading;
    double gyroZ;

    getIMU(&heading, &gyroZ);

    move_motor(speed, targetAngle, heading, gyroZ, targetHeading);

    SerialPC.println(gyroZ);

    kick();

    if (!digitalRead(Pause)) {
      gameFlag = false;
      setMotor(0, FL_FWD, FL_REV);
      setMotor(0, BL_FWD, BL_REV);
      setMotor(0, BR_FWD, BR_REV);
      setMotor(0, FR_FWD, FR_REV);
      for (int i = 0; i < 8; i++) {
        digitalWrite(LED[i], LOW);
      }
    }
    delay(1);
  }else{
    lcd_menu();
    delay(5);
  }
}

void TIM2_Init(void) {

  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;   // 84MHz → 1MHz
  htim2.Init.Period = 499;     // 1MHz / 500 = 2kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  HAL_TIM_Base_Init(&htim2);

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  HAL_TIM_Base_Start(&htim2);
}

void ADC1_DMA_Init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  // PC0, PC1, PC2, PC3, PC4, PC5
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                        GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // PB0, PB1
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = SENSOR_CH;
  hadc1.Init.DMAContinuousRequests = ENABLE;

  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef sConfig;
  uint32_t channels[SENSOR_CH] = {
    ADC_CHANNEL_10,
    ADC_CHANNEL_12,
    ADC_CHANNEL_11,
    ADC_CHANNEL_13,
    ADC_CHANNEL_14,
    ADC_CHANNEL_8,
    ADC_CHANNEL_15,
    ADC_CHANNEL_9
  };

  for(int i=0;i<SENSOR_CH;i++)
  {
    sConfig.Channel = channels[i];
    sConfig.Rank = i+1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  }

  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void ADC2_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;

  HAL_ADC_Init(&hadc2);

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;

  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

void processADC() {
  for(int ch=0; ch<SENSOR_CH; ch++) {
    uint32_t sum = 0;
    for(int i=0;i<SAMPLE_NUM;i++)
      sum += adc_buf[ch + i*SENSOR_CH];

    sensor_avg[ch] = sum / SAMPLE_NUM;
  }
}

uint16_t readCatch() {
  HAL_ADC_Start(&hadc2);
  HAL_ADC_PollForConversion(&hadc2, 10);
  uint16_t val = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Stop(&hadc2);
  return val;
}

double wrapAngle180(double angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void writeEEPROM(int addr, byte data) {
  Wire2.beginTransmission(EEPROM_ADDR);
  Wire2.write((uint8_t)(addr >> 8));   // 上位アドレス
  Wire2.write((uint8_t)(addr & 0xFF)); // 下位アドレス
  Wire2.write(data);
  Wire2.endTransmission();
  delay(5); // EEPROM 書き込み待ち
}

byte readEEPROM(int addr) {
  Wire2.beginTransmission(EEPROM_ADDR);
  Wire2.write((uint8_t)(addr >> 8));
  Wire2.write((uint8_t)(addr & 0xFF));
  if (Wire2.endTransmission() != 0) {
    return 0xFF; // I2Cエラー
  }

  Wire2.requestFrom(EEPROM_ADDR, (uint8_t)1);
  if (Wire2.available()) {
    return Wire2.read();
  }
  return 0xFF; // 読めなかった
}

Ball getBall() {
  double X = 0;
  double Y = 0;
  double distance_sum = 0;

  uint16_t min_val = sensor_avg[0];
  uint16_t max_val = sensor_avg[0];

  // min / max 取得（角度計算用）
  for(int i = 0; i < SENSOR_CH; i++){
    if(sensor_avg[i] < min_val) min_val = sensor_avg[i];
    if(sensor_avg[i] > max_val) max_val = sensor_avg[i];
  }

  for(int i = 0; i < SENSOR_CH; i++){

    // ===== 角度用 weight（正規化） =====
    double val = (double)(max_val - sensor_avg[i]) / (max_val - min_val + 1);

    double weight_angle = val * val;

    double rad = BALLANGLE[i] * PI / 180.0;

    X += weight_angle * cos(rad);
    Y += weight_angle * sin(rad);

    // ===== 距離用 weight（生強度） =====
    double weight_dist = 4095 - sensor_avg[i];

    distance_sum += weight_dist;
  }

  Ball ballinfo;

  // ----- 角度 -----
  double angle = atan2(Y, X) * 180.0 / PI;
  angle = wrapAngle180(angle);

  ballinfo.Angle = angle;

  // ----- 距離 -----
  distance_sum /= 100.0;
  ballinfo.Distance = distance_sum;

  return ballinfo;
}

bool getLineSensorValues(bool lineSensor[19]) {
  // 初期化
  for (int i = 0; i < 19; i++) lineSensor[i] = false;

  SerialLine.write(LINE_SENSOR_INFO); // センサに情報要求
  unsigned long startTime = millis();

  // 10ms以内にヘッダ＋3バイトが来るのを待つ
  while (SerialLine.available() < 4) {
      if (millis() - startTime > 10) {
          // タイムアウト
          return false;
      }
  }

  // ヘッダ読み込み
  byte header = SerialLine.read();
  if (header != LINE_SENSOR_HEADER) {
      // ヘッダ不一致
      return false;
  }

  // センサデータ3バイト読み込み
  byte buffer[3];
  SerialLine.readBytes(buffer, 3);

  uint32_t bits = ((uint32_t)(buffer[2] & 0x07) << 16) | // 上位3bit
                  ((uint32_t)buffer[1] << 8) |
                  buffer[0];

  // 19センサ分に展開
  for (int i = 0; i < 19; i++) {
      lineSensor[i] = (bits >> i) & 0x01;
  }

  return true; // 正常終了
}

void setLineThreshold(uint8_t threshold) {
  SerialLine.write(LINE_SetThreshold);
  SerialLine.write(threshold);
}

void saveLineThreshold(uint8_t threshold){
  writeEEPROM(EE_LINE_THRESHOLD, threshold);
}

uint8_t loadLineThreshold() {
  uint8_t val = readEEPROM(EE_LINE_THRESHOLD);

  // 未書き込み or エラー対策
  if (val == 0xFF) {
    return line_threshold; // デフォルト値（今使っている仮値）
  }
  return val;
}

double getHeading() {
  sensors_event_t ev;
  bno.getEvent(&ev);

  double raw = ev.orientation.x;   // 0-360
  double heading = raw - headingOffset;

  return wrapAngle180(heading);
}

double getGyroZ() {
  sensors_event_t gyro;
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

  return gyro.gyro.z * 57.2958;
}

void getIMU(double *heading, double *gyroZ) {
  sensors_event_t ev, gyro;
  bno.getEvent(&ev);
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);

  float raw = ev.orientation.x;
  *heading = wrapAngle180((double)raw - headingOffset);
  *gyroZ = gyro.gyro.z * 57.2958; // deg/s
}

void resetHeadingZero() {
  sensors_event_t ev;
  bno.getEvent(&ev);
  double raw = wrapAngle180(ev.orientation.x);
  headingOffset = raw;
}

int16_t getLineAngle() {
  int16_t value = 0;
  SerialLine.write(LINE_ANGLE_INFO);
  unsigned long timeout = millis();
  while (SerialLine.available() < 2) {  // int16_t は 2 バイト
    if (millis() - timeout > 10) {      // 10msタイムアウト
      return -1;  // タイムアウト時は -1 を返す
    }
  }

  byte buffer[2];
  SerialLine.readBytes(buffer, 2);
  memcpy(&value, buffer, 2);

  return value;
}

void setMotor(int pwm, int MDpin1, int MDpin2) {
  pwm = constrain(pwm,-PWM_limit,PWM_limit);
  if (pwm > 0) {
    analogWrite(MDpin1, pwm);
    analogWrite(MDpin2, 0);
  } else if (pwm < 0) {
    analogWrite(MDpin1, 0);
    analogWrite(MDpin2, -pwm);
  } else {
    analogWrite(MDpin1, 0);
    analogWrite(MDpin2, 0);
  }
}

void move_motor(int speed, double target_angle, double heading, double gyroZ, double tarHeading) {
  unsigned long currentTime = micros();
  double dt = (currentTime - preTime) / 1000000.0;
  if (dt < 0.001) return;
  preTime = currentTime;

  P = wrapAngle180(tarHeading - heading);

  I += P * dt;
  I = constrain(I, -180.0, 180.0);

  double dHeading = wrapAngle180(heading - preHeading);
  double rawpassD = -dHeading / dt;
  D = 0.9 * D + 0.1 * rawpassD;
  preHeading = heading;
  preP = P;

  double PID = Kp * P + Kd * D;

  //double gyroLPF = 0;

  //gyroLPF = gyroLPF * 0.8 + gyroZ * 0.2;
  //double PID = Kp * P - Kd * gyroLPF;

  double omega = PID;

  if (abs(omega) < 4.0) omega = 0;
  if (abs(P) < 2.0 && abs(gyroZ) < 2.0) omega = 0;

  /*
  SerialPC.print(">P:");
  SerialPC.println(P);
  SerialPC.print(">D:");
  SerialPC.println(D);
  SerialPC.print(">PID:");
  SerialPC.println(PID);
  */

  int m_fr = (int)(speed * -cos(radians(target_angle + 45.0)));
  int m_br = (int)(speed * -cos(radians(target_angle - 45.0)));
  int m_bl = (int)(speed * cos(radians(target_angle + 45.0)));
  int m_fl = (int)(speed * cos(radians(target_angle - 45.0)));

  int v_fr = m_fr + (int)omega;
  int v_br = m_br + (int)omega;
  int v_bl = m_bl + (int)omega;
  int v_fl = m_fl + (int)omega;

  /*
  int v_fr = (int)omega;
  int v_br = (int)omega;
  int v_bl = (int)omega;
  int v_fl = (int)omega;
  */

  v_fr = constrain(v_fr, -255, 255);
  v_br = constrain(v_br, -255, 255);
  v_bl = constrain(v_bl, -255, 255);
  v_fl = constrain(v_fl, -255, 255);

  setMotor(v_fr, FR_FWD, FR_REV);
  setMotor(v_br, BR_FWD, BR_REV);
  setMotor(v_bl, BL_FWD, BL_REV);
  setMotor(v_fl, FL_FWD, FL_REV);
}

void motor_test(){
  for(int i = 0; i < 8; i++){
  for (int pwm = 0; pwm <= 255; pwm++) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(1);
  }
  delay(10);
  for (int pwm = 255; pwm >= 0; pwm--) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(1);
  }
  delay(10);
  for (int pwm = 0; pwm >= -255; pwm--) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(1);
  }
  delay(10);
  for (int pwm = -255; pwm <= 0; pwm++) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(1);
  }
  delay(10);
  setMotor(0, FL_FWD, FL_REV);
  setMotor(0, FR_FWD, FR_REV);
  setMotor(0, BL_FWD, BL_REV);
  setMotor(0, BR_FWD, BR_REV);
}
}

void kick() {
  static bool catching = false;
  static uint32_t lastCatchTime = 0;
  uint16_t catchval = readCatch();
  if (catchval < 200) { //ボール保持判定
    if (!catching) {
      lastCatchTime = millis();
      catching = true;
    }
    if (catching && millis() - lastCatchTime > 10 && millis() - lastCatchTime < 100) {
      digitalWrite(Kick, HIGH);
    } else if(millis() - lastCatchTime >= 100) {
      digitalWrite(Kick, LOW);
      catching = false;
    }
  } else {
    catching = false;
    digitalWrite(Kick, LOW);
  }
}

void lcd_drawarrow(double angle) {
  const int cx = 64;   // 画面中央 (128/2)
  const int cy = 32;   // 画面中央 (64/2)
  const int r_body = 8;
  const int r_arrow = 20;

  // ロボット本体（円）
  display.drawCircle(cx, cy, r_body, SSD1306_WHITE);
  display.fillCircle(cx, cy, 2, SSD1306_WHITE); // 中心点

  // 角度 → ラジアン（上が0°）
  double rad = radians(angle - 90);

  // 矢印先端
  int x2 = cx + r_arrow * cos(rad);
  int y2 = cy + r_arrow * sin(rad);

  // 矢印の軸
  display.drawLine(cx, cy, x2, y2, SSD1306_WHITE);

  // 矢印の羽
  double headAngle = radians(25);
  int hx1 = x2 - 6 * cos(rad - headAngle);
  int hy1 = y2 - 6 * sin(rad - headAngle);
  int hx2 = x2 - 6 * cos(rad + headAngle);
  int hy2 = y2 - 6 * sin(rad + headAngle);

  display.drawLine(x2, y2, hx1, hy1, SSD1306_WHITE);
  display.drawLine(x2, y2, hx2, hy2, SSD1306_WHITE);
}

void lcd_drawLineSensors(bool lineSensor[19]) {
  const int cx = 64;
  const int cy = 32;
  const int r_ring = 20;  // 内側円の半径
  const int r_outer = 30; // 外側センサ用半径

  // エンジェルリング（1〜16）
  for (int i = 0; i < 16; i++) {
      float angle = radians(270 - i * (360.0 / 16.0));
      int x = cx + r_ring * cos(angle);
      int y = cy + r_ring * sin(angle);
      if (lineSensor[i]) {
          display.fillCircle(x, y, 2, SSD1306_WHITE); // 反応ありは塗りつぶし
      } else {
          display.drawCircle(x, y, 2, SSD1306_WHITE); // 反応なしは輪郭だけ
      }
  }

  // 外側センサ（17〜19）
  // 左（17）
  if (lineSensor[16]) {
      display.fillRect(cx - r_outer -1, cy - 6, 3, 10, SSD1306_WHITE);
  } else {
      display.drawRect(cx - r_outer -1, cy - 6, 3, 10, SSD1306_WHITE);
  }

  // 後（18）
  if (lineSensor[17]) {
    display.fillRect(cx -4 , cy + r_outer - 3, 8, 3, SSD1306_WHITE);
  } else {
    display.drawRect(cx - 4, cy + r_outer - 3, 8, 3, SSD1306_WHITE);
  }

  // 右（19）
  if (lineSensor[18]) {
    display.fillRect(cx + r_outer -1, cy - 6, 3, 10, SSD1306_WHITE);
  } else {
    display.drawRect(cx + r_outer -1, cy - 6, 3, 10, SSD1306_WHITE);
  }
}

void lineCalibration() {
  bool sensorValues[19];
  line_threshold = 0;  // 初期しきい値
  bool success;

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Running...");
  display.display();

  for (line_threshold = 0; line_threshold <= 255; line_threshold++) {
      setLineThreshold(line_threshold);      // ラインセンサ側に送信
      delay(20);                           // 設定が反映されるまで少し待つ

      // ラインセンサ値を取得f
      success = getLineSensorValues(sensorValues);

      if (!success) {  // 応答なし
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Failed");
          display.display();
          delay(1000);
          return; // 関数終了、thresholdは変更せず
      }

      // センサが全て反応しなくなったか確認
      bool allOff = true;
      for (int i = 0; i < 19; i++) {
          if (sensorValues[i] == 1) {
              allOff = false;
              if(line_threshold < 253){
                line_threshold = line_threshold + 1; //ちらつき防止
              }
              setLineThreshold(line_threshold);    // ラインセンサ側に送信
              saveLineThreshold(line_threshold);  // EEPROMに保存
              break;
          }
      }

      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Running...");
      display.setCursor(0,8);
      display.print("Threshold:");
      display.println(line_threshold);
      display.display();

      if (allOff) {
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Success");
          display.setCursor(0,8);
          display.print("Threshold:");
          display.println(line_threshold);
          display.display();
          delay(1000);
          break;
      }
  }
}

void lcd_menu(){
  static int menu = 0;
  static int cursor = 0;

  static bool prevEnter = true;
  static bool prevNext  = true;
  static bool prevBack  = true;
  static bool prevOpt   = true;

  bool nowEnter = digitalRead(Enter);
  bool nowNext  = digitalRead(Next);
  bool nowBack  = digitalRead(Back);
  bool nowOpt   = digitalRead(Option);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  //画面処理
  if(menu == 0){
    display.setCursor(0, cursor * 8);
    display.print(">");

    display.setCursor(10,0);
    display.println("Start Game");
    display.setCursor(10,8);
    display.println("Debug");
    display.setCursor(10,16);
    display.println("Settings");

  }
  if(menu == 10){
    display.setCursor(0, cursor * 8);
    display.print(">");

    display.setCursor(10,0);
    display.println("line_test");
    display.setCursor(10,8);
    display.println("ball_test");
    display.setCursor(10,16);
    display.println("motor_test");
    display.setCursor(10,24);
    display.println("kicker_test");
    display.setCursor(10,32);
    display.println("BNO055_test");
    display.setCursor(10,40);
    display.println("Back");
  }
  if(menu == 11){
    display.setCursor(0,0);
    display.print("MoveAngle:");
    display.println(getLineAngle());
    display.setCursor(10,8);
    display.println(">Back");
    display.display();
  }
  if(menu == 12){
    Ball ball = getBall();
    display.clearDisplay();
    lcd_drawarrow(ball.Angle);
    display.setCursor(0,0);
    display.print("Angle:");
    display.print(ball.Angle);
    display.setCursor(0,8);
    display.print(" Distance:");
    display.println(ball.Distance);
    display.setCursor(10,16);
    display.println(">Back");
    display.display();
  }
  if(menu == 13){
    display.setCursor(0, cursor * 8);
    display.print(">");

    display.setCursor(10,0);
    display.println("Run");
    display.setCursor(10,8);
    display.println("Back");
  }
  if(menu == 14){
    display.setCursor(0, cursor * 8);
    display.print(">");

    display.setCursor(10,0);
    display.println("Run");
    display.setCursor(10,8);
    display.println("Back");
  }
  if(menu == 15){
    display.setCursor(10,0);
    display.println(getHeading());
    display.setCursor(10,8);
    display.println(">Back");
  }
  if(menu == 20){
    display.setCursor(0, cursor * 8);
    display.print(">");
    display.setCursor(10,0);
    display.println("line_caribration");
    display.setCursor(10,8);
    display.println("line_setting");
    display.setCursor(10,16);
    display.println("line_check");
    display.setCursor(10,24);
    display.println("BNO055_reset");
    display.setCursor(10,32);
    display.println("Back");
  }
  if(menu == 21){
    display.setCursor(0, cursor * 8);
    display.print(">");
    display.setCursor(10,0);
    display.println("Run");
    display.setCursor(10,8);
    display.println("Back");
  }
  if(menu == 22){
    display.setCursor(0, cursor * 8 + 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("Threshold:");
    display.println(line_threshold);
    display.setCursor(10,8);
    display.println("up");
    display.setCursor(10,16);
    display.println("down");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 23){
    display.setCursor(10,0);
    display.println(">Back");
    bool sensorValues[19] = {0};
    getLineSensorValues(sensorValues);
    lcd_drawLineSensors(sensorValues);
  }
  if(menu == 24){
    display.setCursor(0, cursor * 8);
    display.print(">");
    display.setCursor(10,0);
    display.println("Reset");
    display.setCursor(10,8);
    display.println("Back");
  }

  // ボタン処理
  if(prevEnter && !nowEnter){
    if(menu == 0){
      if(cursor == 0){
        gameFlag = true;
      }
      else if(cursor == 1){
        menu = 10;
        cursor = 0;
      }
      else if(cursor == 2){
        menu = 20;
        cursor = 0;
      }
    }
    else if(menu == 10){
      if(cursor == 0){
        menu = 11;
        cursor = 0;
      }
      else if(cursor == 1){
        menu = 12;
        cursor = 0;
      }
      else if(cursor == 2){
        menu = 13;
        cursor = 0;
      }
      else if(cursor == 3){
        menu = 14;
        cursor = 0;
      }
      else if(cursor == 4){
        menu = 15;
        cursor = 0;
      }
      else if(cursor == 5){
        menu = 0;
        cursor = 0;
      }
    }
    else if(menu == 11){
      menu = 10;
      cursor = 0;
    }
    else if(menu == 12){
      menu = 10;
      cursor = 0;
    }
    else if(menu == 13){
      if(cursor == 0){
        display.setCursor(10,0);
        display.println("Running...");
        display.setCursor(10,8);
        display.println("Back");
        display.display();
        motor_test();
      }
      if(cursor == 1){
        setMotor(0, FL_FWD, FL_REV);
        setMotor(0, FR_FWD, FR_REV);
        setMotor(0, BL_FWD, BL_REV);
        setMotor(0, BR_FWD, BR_REV);
        menu = 10;
        cursor = 0;
      }
    }
    else if(menu == 14){
      if(cursor == 0){
        display.setCursor(10,0);
        display.println("Running...");
        display.setCursor(10,8);
        display.println("Back");
        display.display();
        digitalWrite(Kick, HIGH);
        delay(100);
        digitalWrite(Kick, LOW);
      }
      if(cursor == 1){
        menu = 10;
        cursor = 0;
      }
    }
    else if(menu == 15){
      menu = 10;
      cursor = 0;
    }
    else if(menu == 20){
      if(cursor == 0){
        menu = 21;
        cursor = 0;
      }
      else if(cursor == 1){
        menu = 22;
        cursor = 0;
      }
      else if(cursor == 2){
        menu = 23;
        cursor = 0;
      }
      else if(cursor == 3){
        menu = 24;
        cursor = 0;
      }
      else if(cursor == 4){
        menu = 0;
        cursor = 0;
      }
    }
    else if(menu == 21){
      if(cursor == 0){
        lineCalibration();
        menu = 20;
        cursor = 0;
      }
      else if(cursor == 1){
        menu = 20;
        cursor = 0;
      }
    }
    else if(menu == 22){
      if(cursor == 0){
        line_threshold++;
        if(line_threshold > 255) line_threshold = 255;
        setLineThreshold(line_threshold);
        saveLineThreshold(line_threshold);
      }else if(cursor == 1){
        line_threshold--;
        if(line_threshold < 0) line_threshold = 0;
        setLineThreshold(line_threshold);
        saveLineThreshold(line_threshold);
      }else if(cursor == 2){
        menu = 20;
        cursor = 0;
      }
    }
    else if(menu == 23){
      menu = 20;
      cursor = 0;
    }
    else if(menu == 24){
      if(cursor == 0){
        resetHeadingZero();
        display.clearDisplay();
        display.setCursor(10,0);
        display.println("Done");
        display.display();
        delay(1000);
        menu = 0;
        cursor = 0;
      }
      else if(cursor == 1){
        menu = 20;
        cursor = 0;
      }
    }
  }

  //カーソル移動
  if(prevNext && !nowNext){
    cursor++;
    if(menu == 0 && cursor > 2) cursor = 0;
    if(menu == 10 && cursor > 5) cursor = 0;
    if(menu == 11 && cursor > 0) cursor = 0;
    if(menu == 12 && cursor > 0) cursor = 0;
    if(menu == 13 && cursor > 1) cursor = 0;
    if(menu == 14 && cursor > 1) cursor = 0;
    if(menu == 15 && cursor > 0) cursor = 0;
    if(menu == 20 && cursor > 4) cursor = 0;
    if(menu == 21 && cursor > 1) cursor = 0;
    if(menu == 22 && cursor > 2) cursor = 0;
    if(menu == 23 && cursor > 1) cursor = 0;
    if(menu == 24 && cursor > 0) cursor = 0;
  }

  if(prevBack && !nowBack){
    cursor--;
    if(menu == 0 && cursor < 0) cursor = 2;
    if(menu == 10 && cursor < 0) cursor = 5;
    if(menu == 11 && cursor < 0) cursor = 0;
    if(menu == 12 && cursor < 0) cursor = 0;
    if(menu == 13 && cursor < 0) cursor = 1;
    if(menu == 14 && cursor < 0) cursor = 1;
    if(menu == 15 && cursor < 0) cursor = 0;
    if(menu == 20 && cursor < 0) cursor = 4;
    if(menu == 21 && cursor < 0) cursor = 1;
    if(menu == 22 && cursor < 0) cursor = 2;
    if(menu == 23 && cursor < 0) cursor = 1;
    if(menu == 24 && cursor < 0) cursor = 0;
  }


  if(prevOpt && !nowOpt){
    menu = 10;
    cursor = 0;
  }

  // 状態更新
  prevEnter = nowEnter;
  prevNext  = nowNext;
  prevBack  = nowBack;
  prevOpt   = nowOpt;

  display.display();
}