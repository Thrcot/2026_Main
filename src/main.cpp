#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <display.h>


#include "math.h"

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
#define FL_FWD PC6
#define FL_REV PC7
// BL
#define BL_FWD PC8
#define BL_REV PC9
// BR
#define BR_FWD PA8
#define BR_REV PA9
// FR
#define FR_FWD PA10
#define FR_REV PA11
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

// MARK: constants
bool gameFlag = false;
uint8_t line_threshold = 155; //デフォルト(仮)

int PWM_limit = 245; //0~255
int MAX_PWM_step = 20; //0~255(非推奨)

int speed = 170; //0~255
int PID_limit = 60; //0~255
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0.6;

double headingOffset = 0.0;

// MARK: function
double wrapAngle180(double angle);
void writeEEPROM(int addr, byte data);
byte readEEPROM(int addr);
double getBallAngle();
double getHeading();
int16_t getLineAngle();
void motor_test();
void setMotor(int pwm, int MDpin1, int MDpin2);
void move_motor(int speed, double target_angle, double heading, double tarHeading);
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

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);

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
  pinMode(ADCCatch, INPUT);

  digitalWrite(Kick, LOW);

  pinMode(Back, INPUT);
  pinMode(Pause, INPUT);
  pinMode(Enter, INPUT);
  pinMode(Next, INPUT);
  pinMode(Option, INPUT);

  // bno setup
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

  // line sensor setup
  line_threshold = loadLineThreshold();
  setLineThreshold(line_threshold);      // ラインセンサ側に送信

  delay(1000);
  display.clearDisplay();
  display.display();
}


// MARK: main loop
void loop() {
  if(gameFlag == true){
    static int starttime = millis();
    if(millis() - starttime > 10000){
      setMotor(0, FL_FWD, FL_REV);
      setMotor(0, BL_FWD, BL_REV);
      setMotor(0, BR_FWD, BR_REV);
      setMotor(0, FR_FWD, FR_REV);
      delay(500);
      starttime = millis();
    }


    static int16_t lastLineAngle = -1;  
    static unsigned long lastLineTime = 0;
    static int lineAngle = -1;
    double targetAngle = getBallAngle();
    double heading = getHeading();
    targetAngle = wrapAngle180(targetAngle * 1.3);  //回り込み

    display.clearDisplay();
    lcd_drawarrow(targetAngle);
    display.display();



  lineAngle = getLineAngle();
  if(lineAngle != -1){
    lastLineAngle = lineAngle;
    lastLineTime = millis();
  }

  if(lastLineAngle != -1 && (millis() - lastLineTime) < 500){ // 200ms間は回避
    targetAngle = wrapAngle180((double)lastLineAngle);
  }

    move_motor(speed, targetAngle, heading, 0.0);

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

double getBallAngle() {
  double x = 0.0;
  double y = 0.0;

  // センサ角
  const double sensorAngle[8] = {
    0, 45, 90, 135, 180, 225, 270, 315
  };

  int sensorval[8];
  int minval = 1023;
  // センサ値取得と最もボールに近いセンサを探す
  for (int i = 0; i < 8; i++) {
    sensorval[i] = analogRead(BALL[i]);
    if (sensorval[i] < minval) minval = sensorval[i];
  }

  // 重み付け
  for (int i = 0; i < 8; i++) {
    double w = (double)(1023 - sensorval[i]);
    if (w < 0) w = 0;

    double rad = radians(sensorAngle[i]);
    x += w * cos(rad);
    y += w * sin(rad);
  }
  double angle = atan2(y, x) * 180.0 / M_PI;
  return wrapAngle180(angle);
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
  double raw = wrapAngle180(ev.orientation.x);
  double heading = wrapAngle180(raw - headingOffset);
  return heading;
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

void move_motor(int speed, double target_angle, double heading, double tarHeading) {
  //PID
  static unsigned long prevTime = 0;
  static double prevErr = 0.0;
  static int prev_fr = 0, prev_br = 0, prev_bl = 0, prev_fl = 0;
  double PID = 0.0;

  unsigned long currentTime = micros();
  double dt = (currentTime - prevTime) / 1000000.0;

  double err = wrapAngle180(tarHeading - heading);
  double dErr =  (err - prevErr) / dt;
  prevErr = err;
  prevTime = currentTime;
  PID = Kp * err + Kd * dErr;
  PID = constrain(PID, -PID_limit, PID_limit);

  //Ball
  int m_fr = (int)(speed * -cos(radians(target_angle + 45.0)));
  int m_br = (int)(speed * -cos(radians(target_angle - 45.0)));
  int m_bl = (int)(speed * cos(radians(target_angle + 45.0)));
  int m_fl = (int)(speed * cos(radians(target_angle - 45.0)));

  /*
  //最も大きい移動成分を取得
  double max_move = max(
    max(fabs(m_fl), fabs(m_fr)),
    max(fabs(m_bl), fabs(m_br))
  );

  // 余った成分をPIDに回す
  double pid_margin = 1.0 - max_move;
  pid_margin = constrain(pid_margin, 0.0, 1.0);
  PID = PID * pid_margin;
  */

  //Rotation
  int v_fr = m_fr + PID;
  int v_br = m_br + PID;
  int v_bl = m_bl + PID;
  int v_fl = m_fl + PID;

  v_fr = constrain(v_fr, -255, 255);
  v_br = constrain(v_br, -255, 255);
  v_bl = constrain(v_bl, -255, 255);
  v_fl = constrain(v_fl, -255, 255);

  //加速度制限を追加
  // v_fr
  if(prev_fr * v_fr < 0){        // 符号反転なら一旦停止
    v_fr = 0;
  } else if(v_fr > prev_fr + MAX_PWM_step){
    v_fr = prev_fr + MAX_PWM_step;
  } else if(v_fr < prev_fr - MAX_PWM_step){
    v_fr = prev_fr - MAX_PWM_step;
  }
  // v_br
  if(prev_br * v_br < 0){
    v_br = 0;
  } else if(v_br > prev_br + MAX_PWM_step){
    v_br = prev_br + MAX_PWM_step;
  } else if(v_br < prev_br - MAX_PWM_step){
    v_br = prev_br - MAX_PWM_step;
  }
  // v_bl
  if(prev_bl * v_bl < 0){
    v_bl = 0;
  } else if(v_bl > prev_bl + MAX_PWM_step){
    v_bl = prev_bl + MAX_PWM_step;
  } else if(v_bl < prev_bl - MAX_PWM_step){
    v_bl = prev_bl - MAX_PWM_step;
  }
  // v_fl
  if(prev_fl * v_fl < 0){
    v_fl = 0;
  } else if(v_fl > prev_fl + MAX_PWM_step){
    v_fl = prev_fl + MAX_PWM_step;
  } else if(v_fl < prev_fl - MAX_PWM_step){
    v_fl = prev_fl - MAX_PWM_step;
  }

  setMotor(v_fr, FR_FWD, FR_REV);
  setMotor(v_br, BR_FWD, BR_REV);
  setMotor(v_bl, BL_FWD, BL_REV);
  setMotor(v_fl, FL_FWD, FL_REV);
  prev_fl = v_fl;
  prev_fr = v_fr;
  prev_bl = v_bl;
  prev_br = v_br;
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
  if (analogRead(ADCCatch) < 200) { //ボール保持判定
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
    double ballangle = getBallAngle();
    display.clearDisplay();
    lcd_drawarrow(ballangle);
    display.setCursor(0,0);
    display.print("Angle:");
    display.print(ballangle);
    display.setCursor(10,8);
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
