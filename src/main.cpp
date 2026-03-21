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

#define EE_LINE_THRESHOLD 0x0000  //EEPROMアドレス、1バイト保存
#define EE_SPEED 0x0001  //EEPROMアドレス、1バイト保存
#define EE_PID_KP 0x0002  //EEPROMアドレス、8バイト保存
#define EE_PID_KI 0x000A  //EEPROMアドレス、8バイト保存
#define EE_PID_KD 0x0012  //EEPROMアドレス、8バイト保存

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
#define LINE_RESET 0xB0

// DMA Setting
#define SENSOR_CH 8
#define SAMPLE_NUM 128

// HALハンドラ
ADC_HandleTypeDef hadc1; //DMA
ADC_HandleTypeDef hadc2; //Kicker
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

// MARK: constants
// DMA Buffer
uint16_t adc_buf[SENSOR_CH * SAMPLE_NUM];
volatile uint16_t sensor_avg[SENSOR_CH];
volatile bool adc_ready = false;

struct Ball {
  double Angle;
  double Distance;
  double X;
  double Y;
};

const int BALL[8] = {A1, A2, A3, A4, A5, A6, A7, A8};
double BALLANGLE[8] = {0, 45, 90, 135, 180, 225, 270, 315};

#define RING_LINE 16

constexpr int16_t kAccelOffsetX = -1;
constexpr int16_t kAccelOffsetY = -14;
constexpr int16_t kAccelOffsetZ = -15;
constexpr int16_t kMagOffsetX = 160;
constexpr int16_t kMagOffsetY = 203;
constexpr int16_t kMagOffsetZ = -13;
constexpr int16_t kGyroOffsetX = -1;
constexpr int16_t kGyroOffsetY = -2;
constexpr int16_t kGyroOffsetZ = -2;
constexpr int16_t kAccelRadius = 1000;
constexpr int16_t kMagRadius = 1267;

bool gameFlag = false;
bool ImAttacker = true; // true: Attacker, false: Keeper
uint8_t line_threshold = 155;

int PWM_limit = 250; // yukuyukuMD MAX is 250.

int basespeed = 80;
double Kp = 0.5;
double Ki = 0.0;
double Kd = 0.05;
double preTime = 0.0;
double preHeading = 0.0;
double P = 0.0, I = 0.0, D = 0.0, preP = 0.0;

double KP_ball = 0.2;
double KD_ball = 0.03;

double headingOffset = 0.0;
double prevTime = 0.0;

// Reset ID
enum ResetCause {
  CAUSE_UNKNOWN = 0,
  CAUSE_LOW_POWER = 1,
  CAUSE_WWDG = 2,
  CAUSE_IWDG = 3,
  CAUSE_SOFTWARE = 4,
  CAUSE_POWER_ON = 5,
  CAUSE_EXTERNAL = 6,
  CAUSE_BROWNOUT = 7
};

int resetCause = 0;

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
int16_t getLineAngle();
void motor_test();
void setMotor(int pwm, int MDpin1, int MDpin2);
void move_motor(int speed, double target_angle, double heading, double tarHeading);
void kick();
void lcd_drawarrow(double angle);
void lcd_drawLineSensors(bool lineSensor[RING_LINE]);
void lcd_menu();
bool getLineSensorValues(bool lineSensor[RING_LINE]);
void lineCalibration();
void resetLineSensor();
void resetHeadingZero();
void setLineThreshold(uint8_t threshold);
void saveLineThreshold(uint8_t threshold);
uint8_t loadLineThreshold();
void saveSpeed(uint8_t speed);
uint8_t loadSpeed();
void savePIDgain(double Kp, double Ki, double Kd);
void loadPIDgain(double *Kp, double *Ki, double *Kd);
int getResetCause();

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

  // shirokumaShiki bno
  adafruit_bno055_offsets_t offsets;
  offsets.accel_offset_x = kAccelOffsetX;
  offsets.accel_offset_y = kAccelOffsetY;
  offsets.accel_offset_z = kAccelOffsetZ;
  offsets.gyro_offset_x = kGyroOffsetX;
  offsets.gyro_offset_y = kGyroOffsetY;
  offsets.gyro_offset_z = kGyroOffsetZ;
  offsets.mag_offset_x = kMagOffsetX;
  offsets.mag_offset_y = kMagOffsetY;
  offsets.mag_offset_z = kMagOffsetZ;
  offsets.accel_radius = kAccelRadius;
  offsets.mag_radius = kMagRadius;
  bno.setSensorOffsets(offsets);

  bno.setExtCrystalUse(true);
  //bno.setMode(OPERATION_MODE_IMUPLUS);

  for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], LOW);
  }
  delay(500);
  resetHeadingZero();

  SerialPC.println("[Debug] Line initialize");
  // line sensor setup
  line_threshold = loadLineThreshold();
  setLineThreshold(line_threshold);      // ラインセンサ側に送信

  // Speed setup
  basespeed = loadSpeed();

  // PID gain setup
  loadPIDgain(&Kp, &Ki, &Kd);

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

  SerialPC.println("[Debug] Getting reset cause");
  resetCause = getResetCause();
  SerialPC.println("[Debug] Done");

  SerialPC.println("[Debug] Setup end");
}


// MARK: main loop
void loop() {
  if (adc_ready) {
    adc_ready = false;
    processADC();
  }
  if(gameFlag == true){
    if (ImAttacker == true) {
      // Attacker algorithm
      SerialPC.println("[Debug] Game loop");
      static int16_t lastLineAngle = -1;
      static unsigned long lastLineTime = 0;
      static int lineAngle = -1;
      static unsigned long lastHeadingTime = 0;
      static int targetHeading = 0;
      static bool BallIsNear = false;
      static bool ImOnCorner = false;
      double speed = basespeed;

      Ball b = getBall();
      double targetAngle = b.Angle;
      double ballX = b.X;
      double ballY = b.Y;

      static double prevBallErr = 0;
      double ballErr = b.Angle;
      double dBallErr = ballErr - prevBallErr;
      prevBallErr = ballErr;

      double NearThr = 150;
      if (BallIsNear) {
        NearThr = 120;
      } else {
        ;
      }

      if (b.Distance >= NearThr) {
        BallIsNear = true;
        double KP_ball = 0.2;
        double KD_ball = 0.0;
        double k = 80;

        /*
        if (b.Distance >= 180) {
          KP_ball = 0.1;
          KD_ball = 0.01;
          k = 100;
        }
          */

        double rad = b.Angle * PI / 180.0;
        double frontGain = abs(sin(rad));
        double pd = KP_ball * ballErr + KD_ball * dBallErr;

        targetAngle = b.Angle + k * sin(rad) + pd;

        speed = basespeed * (0.7 + 0.3 * abs(cos(rad)));
      } else if (b.Distance >= 10) {
        BallIsNear = false;
        speed = basespeed + 50;
      } else {
        BallIsNear = false;
        speed = 0;
      }

      display.clearDisplay();
      lcd_drawarrow(targetAngle);
      display.display();

      lineAngle = getLineAngle();   //ライン踏んだ時の移動角
      if(lineAngle != -1){
        speed += 30;
        lastLineAngle = lineAngle;
        lastLineTime = millis();
        if (!ImOnCorner) {
          if((lineAngle > 125) && (lineAngle < 145)){
            ImOnCorner = true;
            targetHeading = -73;
            lastHeadingTime = millis();
          }else if((lineAngle > 215) && (lineAngle < 235)){
            ImOnCorner = true;
            targetHeading = 73;
            lastHeadingTime = millis();
          } else {
            ImOnCorner = false;
          }
        }
      } else {

      }

      int backtime = 5;

      if(lastLineAngle != -1 && (millis() - lastLineTime) < backtime){  // backtime msは後退する
        targetAngle = wrapAngle180((double)lastLineAngle);
      }
      if(targetHeading != 0 && (millis() - lastHeadingTime) > 800){ // ライン踏んでから0.8秒後には目標角度をリセット
        targetHeading = 0;
        ImOnCorner = false;
      }

      double heading = getHeading();

      move_motor(speed, targetAngle, heading, targetHeading);
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
    } else {
      // Keeper algorithm(仮)
      SerialPC.println("[Debug] Game loop");
      static int16_t lastLineAngle = -1;
      static unsigned long lastLineTime = 0;
      static int lineAngle = -1;
      double speed = basespeed;
      double targetHeading = 0;
      double targetAngle = 0;

      Ball b = getBall();
      speed = basespeed * fabs(sin(b.Angle * PI / 180.0));
      if(b.Angle > 0 && b.Angle < 180){
        targetAngle = 90;
      }else{
        targetAngle  = -90;
      }

      display.clearDisplay();
      lcd_drawarrow(targetAngle);
      display.display();

      lineAngle = getLineAngle();   //ライン踏んだ時の移動角
      if(lineAngle != -1){
        lastLineAngle = lineAngle;
        lastLineTime = millis();
      }

      if(lastLineAngle != -1 && (millis() - lastLineTime) < 50){  // 50msは後退する
        targetAngle = wrapAngle180((double)lastLineAngle);
      }

      double heading = getHeading();

      move_motor(speed, targetAngle, heading, targetHeading);

      kick();

      if (!digitalRead(Pause)) {
        gameFlag = false;
        resetLineSensor();
        setMotor(0, FL_FWD, FL_REV);
        setMotor(0, BL_FWD, BL_REV);
        setMotor(0, BR_FWD, BR_REV);
        setMotor(0, FR_FWD, FR_REV);
        for (int i = 0; i < 8; i++) {
          digitalWrite(LED[i], LOW);
        }
      }
      delay(1);
    }
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
  ballinfo.X = X;
  ballinfo.Y = Y;

  // ----- 距離 -----
  distance_sum /= 100.0;
  ballinfo.Distance = distance_sum;

  return ballinfo;
}

bool getLineSensorValues(bool lineSensor[RING_LINE]) {
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
  for (int i = 0; i < RING_LINE; i++) {
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

void saveSpeed(uint8_t speed){
  writeEEPROM(EE_SPEED, speed);
}

uint8_t loadSpeed() {
  uint8_t val = readEEPROM(EE_SPEED);

  // 未書き込み or エラー対策
  if (val == 0xFF) {
    return basespeed; // デフォルト値（今使っている仮値）
  }
  return val;
}

void savePIDgain(double Kp, double Ki, double Kd){
  byte *p;

  p = (byte*)&Kp;
  for(int i=0;i<8;i++){
    writeEEPROM(EE_PID_KP + i, p[i]);
  }

  p = (byte*)&Ki;
  for(int i=0;i<8;i++){
    writeEEPROM(EE_PID_KI + i, p[i]);
  }

  p = (byte*)&Kd;
  for(int i=0;i<8;i++){
    writeEEPROM(EE_PID_KD + i, p[i]);
  }
}

void loadPIDgain(double *Kp, double *Ki, double *Kd) {
  double kp_eep;
  double ki_eep;
  double kd_eep;
  byte *p;

  p = (byte*)&kp_eep;
  for(int i=0;i<8;i++){
    p[i] = readEEPROM(EE_PID_KP + i);
  }

  p = (byte*)&ki_eep;
  for(int i=0;i<8;i++){
    p[i] = readEEPROM(EE_PID_KI + i);
  }

  p = (byte*)&kd_eep;
  for(int i=0;i<8;i++){
    p[i] = readEEPROM(EE_PID_KD + i);
  }

  // NaNでなければ反映
  if(!isnan(kp_eep)) *Kp = kp_eep;
  if(!isnan(ki_eep)) *Ki = ki_eep;
  if(!isnan(kd_eep)) *Kd = kd_eep;
}

double getHeading() {
  sensors_event_t ev;
  bno.getEvent(&ev);

  double raw = ev.orientation.x;   // 0-360
  double heading = raw - headingOffset;

  return wrapAngle180(heading);
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

void resetLineSensor() {
  SerialLine.write(LINE_RESET);
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

void move_motor(int speed, double target_angle, double heading, double tarHeading) {

  unsigned long currentTime = micros();
  double dt = (currentTime - preTime) / 1000000.0;
  preTime = currentTime;

  if (dt < 0.001) return;

  P = wrapAngle180(tarHeading - heading);

  I += P * dt;
  I = constrain(I, -180.0, 180.0);

  double dHeading = wrapAngle180(heading - preHeading);
  double rawpassD = -dHeading / dt;
  D = 0.9 * D + 0.1 * rawpassD;
  preHeading = heading;

  double PID = Kp * P + Kd * D;

  if (fabs(P) < 1.5) PID = 0;

  // ---- 並進ベクトル ----
  double tx_fr = -cos(radians(target_angle + 45.0));
  double tx_br = -cos(radians(target_angle - 45.0));
  double tx_bl =  cos(radians(target_angle + 45.0));
  double tx_fl =  cos(radians(target_angle - 45.0));

  double m_fr = speed * tx_fr;
  double m_br = speed * tx_br;
  double m_bl = speed * tx_bl;
  double m_fl = speed * tx_fl;

  // ---- 回転成分 ----
  double omega = PID * (speed / 255.0);

  // ===== 比率リミット =====
  double trans = speed;
  double rot = fabs(omega);

  double sum = trans + rot;

  if (sum > 255.0) {
    double trans_scale = (255.0 * 0.6) / trans;
    double rot_scale   = (255.0 * 0.4) / rot;

    m_fr *= trans_scale;
    m_br *= trans_scale;
    m_bl *= trans_scale;
    m_fl *= trans_scale;

    omega *= rot_scale;
  }

  // ---- 合成 ----
  int v_fr = (int)(m_fr + omega);
  int v_br = (int)(m_br + omega);
  int v_bl = (int)(m_bl + omega);
  int v_fl = (int)(m_fl + omega);

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
  for(int i = 0; i < 2; i++){
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
    delay(10);
  }
  delay(10);
  for (int pwm = 0; pwm >= -255; pwm--) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(10);
  }
  delay(10);
  for (int pwm = -255; pwm <= 0; pwm++) {
    setMotor(pwm, FL_FWD, FL_REV);
    setMotor(pwm, FR_FWD, FR_REV);
    setMotor(pwm, BL_FWD, BL_REV);
    setMotor(pwm, BR_FWD, BR_REV);
    delay(10);
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

void lcd_drawLineSensors(bool lineSensor[RING_LINE]) {
  const int cx = 64;
  const int cy = 32;
  const int r_ring = 20;  // 内側円の半径

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
    display.print("Reset Cause: ");
    display.println(resetCause);
    display.setCursor(10,48);
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
    display.print("Distance:");
    display.println(ball.Distance);
    display.setCursor(0,16);
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
    display.setCursor(0, 8 + cursor * 8);
    display.print(">");

    display.setCursor(10,0);
    display.print("Catch: ");
    uint16_t catchval = readCatch();
    display.print(catchval);
    if (catchval < 200) { //ボール保持判定
      display.print(": 1");
    } else {
      display.print(": 0");
    }

    display.setCursor(10,8);
    display.println("Run");
    display.setCursor(10,16);
    display.println("Back");
  }
  if(menu == 15){
    display.setCursor(10,0);
    display.println(getHeading());
    display.setCursor(10,8);
    display.println(">Back");
  }
  if(menu == 16){
    display.setCursor(0, 0);
    display.print("Cause: ");
    switch(resetCause){
      case 0:
        display.println("Unknown");
        break;
      case 1:
        display.println("Low Power");
        break;
      case 2:
        display.println("WWDG");
        break;
      case 3:
        display.println("IWDG");
        break;
      case 4:
        display.println("Software");
        break;
      case 5:
        display.println("Power On");
        break;
      case 6:
        display.println("External");
        break;
      case 7:
        display.println("Brownout");
        break;
    }
    display.setCursor(0,8);
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
    display.println("speed_setting");
    display.setCursor(10,32);
    display.println("PID_setting");
    display.setCursor(10,40);
    display.println("BNO055_reset");
    display.setCursor(10,48);
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
    bool sensorValues[RING_LINE] = {0};
    getLineSensorValues(sensorValues);
    lcd_drawLineSensors(sensorValues);
  }
  if(menu == 24){
    display.setCursor(0, cursor * 8 + 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("Speed:");
    display.println(basespeed);
    display.setCursor(10,8);
    display.println("up");
    display.setCursor(10,16);
    display.println("down");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 25){
    display.setCursor(0, cursor * 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("set P");
    display.setCursor(10,8);
    display.println("set I");
    display.setCursor(10,16);
    display.println("set D");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 250){
    display.setCursor(0, cursor * 8 + 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("Kp:");
    display.println(Kp, 3);
    display.setCursor(10,8);
    display.println("up");
    display.setCursor(10,16);
    display.println("down");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 251){
    display.setCursor(0, cursor * 8 + 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("Ki:");
    display.println(Ki, 3);
    display.setCursor(10,8);
    display.println("up");
    display.setCursor(10,16);
    display.println("down");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 252){
    display.setCursor(0, cursor * 8 + 8);
    display.print(">");
    display.setCursor(10,0);
    display.print("Kd:");
    display.println(Kd, 3);
    display.setCursor(10,8);
    display.println("up");
    display.setCursor(10,16);
    display.println("down");
    display.setCursor(10,24);
    display.println("Back");
  }
  if(menu == 26){
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
        resetLineSensor();
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
        menu = 16;
        cursor = 0;
      }
      else if(cursor == 6){
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
        display.print("Catch: ");
        uint16_t catchval = readCatch();
        display.print(catchval);
        if (catchval < 200) { //ボール保持判定
          display.print(": 1");
        } else {
          display.print(": 0");
        }

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
        display.setCursor(10,8);
        display.println("Running...");
        display.setCursor(10,16);
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
    else if(menu == 16){
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
        menu = 25;
        cursor = 0;
      }
      else if(cursor == 5){
        menu = 26;
        cursor = 0;
      }
      else if(cursor == 6){
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
        basespeed = basespeed + 5;
        if(basespeed > 255) basespeed = 255;
        saveSpeed(basespeed);
      }else if(cursor == 1){
        basespeed = basespeed - 5;
        if(basespeed < 0) basespeed = 0;
        saveSpeed(basespeed);
      }else if(cursor == 2){
        menu = 20;
        cursor = 0;
      }
    }
    else if(menu == 25){
      if(cursor == 0){
        menu = 250;
      }else if(cursor == 1){
        menu = 251;
      }else if(cursor == 2){
        menu = 252;
      }else if(cursor == 3){
        menu = 20;
        cursor = 0;
      }
    }
    else if(menu == 250){
      if(cursor == 0){
        Kp = Kp + 0.005;
        if(Kp > 1) Kp = 1;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 1){
        Kp = Kp - 0.005;
        if(Kp < 0) Kp = 0;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 2){
        menu = 25;
        cursor = 0;
      }
    }
    else if(menu == 251){
      if(cursor == 0){
        Ki = Ki + 0.005;
        if(Ki > 1) Ki = 1;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 1){
        Ki = Ki - 0.005;
        if(Ki < 0) Ki = 0;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 2){
        menu = 25;
        cursor = 0;
      }
    }
    else if(menu == 252){
      if(cursor == 0){
        Kd = Kd + 0.005;
        if(Kd > 1) Kd = 1;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 1){
        Kd = Kd - 0.005;
        if(Kd < 0) Kd = 0;
        savePIDgain(Kp, Ki, Kd);
      }else if(cursor == 2){
        menu = 25;
        cursor = 0;
      }
    }
    else if(menu == 26){
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
    if(menu == 10 && cursor > 6) cursor = 0;
    if(menu == 11 && cursor > 0) cursor = 0;
    if(menu == 12 && cursor > 0) cursor = 0;
    if(menu == 13 && cursor > 1) cursor = 0;
    if(menu == 14 && cursor > 1) cursor = 0;
    if(menu == 15 && cursor > 0) cursor = 0;
    if(menu == 16 && cursor > 0) cursor = 0;
    if(menu == 20 && cursor > 6) cursor = 0;
    if(menu == 21 && cursor > 1) cursor = 0;
    if(menu == 22 && cursor > 2) cursor = 0;
    if(menu == 23 && cursor > 1) cursor = 0;
    if(menu == 24 && cursor > 2) cursor = 0;
    if(menu == 25 && cursor > 3) cursor = 0;
    if(menu == 250 && cursor > 2) cursor = 0;
    if(menu == 251 && cursor > 2) cursor = 0;
    if(menu == 252 && cursor > 2) cursor = 0;
    if(menu == 26 && cursor > 1) cursor = 0;
  }

  if(prevBack && !nowBack){
    cursor--;
    if(menu == 0 && cursor < 0) cursor = 2;
    if(menu == 10 && cursor < 0) cursor = 6;
    if(menu == 11 && cursor < 0) cursor = 0;
    if(menu == 12 && cursor < 0) cursor = 0;
    if(menu == 13 && cursor < 0) cursor = 1;
    if(menu == 14 && cursor < 0) cursor = 1;
    if(menu == 15 && cursor < 0) cursor = 0;
    if(menu == 16 && cursor < 0) cursor = 0;
    if(menu == 20 && cursor < 0) cursor = 6;
    if(menu == 21 && cursor < 0) cursor = 1;
    if(menu == 22 && cursor < 0) cursor = 2;
    if(menu == 23 && cursor < 0) cursor = 1;
    if(menu == 24 && cursor < 0) cursor = 2;
    if(menu == 25 && cursor < 0) cursor = 3;
    if(menu == 250 && cursor < 0) cursor = 2;
    if(menu == 251 && cursor < 0) cursor = 2;
    if(menu == 252 && cursor < 0) cursor = 2;
    if(menu == 26 && cursor < 0) cursor = 1;
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

int getResetCause() {
  int cause = CAUSE_UNKNOWN;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
    cause = CAUSE_LOW_POWER;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
    cause = CAUSE_WWDG;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    cause = CAUSE_IWDG;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
    cause = CAUSE_SOFTWARE;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
    cause = CAUSE_POWER_ON;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
    cause = CAUSE_EXTERNAL;
  } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
    cause = CAUSE_BROWNOUT;
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();

  return cause;
}