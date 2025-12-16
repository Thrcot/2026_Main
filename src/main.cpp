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

#define SCREEN_ADDR 0x3C
#define BNO055_ADDR 0x28
#define EEPROM_ADDR 0x50

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// define variables
const int PWM_MAX = 255;  //最大値ではなく比率になるので注意
bool gameFlag = false;
unsigned long lastCatchTime = 0;
bool catching = false;

int menu = 0;
const unsigned char memuDisplay[] = {};


// MARK: SPEED
#define SPEED 0.5

#define LINE_INIT 0xAD
#define LINE_INFO 0xAE
#define LINE_OK 0xAF

struct Ball {
  double ANGLE;
  double DISTANCE;
};
const double BALL_THRESHOLD = 280.0;

// MARK: PID
double Kp = 0.02;
double Ki = 0.00025;
double Kd = 0.002;

double errI = 0.0;
double prevErr = 0.0;

unsigned long prevTime = 0;

// define functions
void writeEEPROM(int addr, byte data);
byte readEEPROM(int addr);
void kick();
double culcMoveAngle(double ballAngle);
double getAngle();
double getLine();
Ball getBall();
void setMotor(double power, int pinF, int pibR);
void move_motor(double speed, double moveDeg, double heading, double tarHeading);
void move(double speed, double moveDeg, double heading, double targetHeading);
void motorTest();

void setup() {
  SerialPC.begin(115200);
  SerialLine.begin(115200);
  Wire1.begin();
  Wire2.begin();

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
  while (!bno.begin()) {
    for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], !digitalRead(LED[i]));
  }
  delay(100);
  }
  for (int i = 0; i < 8; i++) {
    digitalWrite(LED[i], LOW);
  }
  bno.setExtCrystalUse(true);

  SerialPC.println("[Debug] Setup end");

  delay(1000);
  display.clearDisplay();
  display.display();
}


// MARK: main loop
void loop() {

  //motorTest();

  if (!digitalRead(Enter)){
    gameFlag = true;
  }

  while (gameFlag) {
    double angle = 0.0;
    double moveAngle = 0.0;
    double speed = 0.0;
    double tarAngle = 0.0;
    double LineAngle = 0.0;
    Ball ball = {0.0, 0.0};

    // read Line

    // read ball
    ball = getBall();
    moveAngle = culcMoveAngle(ball.ANGLE);
    speed = SPEED;

    // read current angle
    angle = getAngle();

    // Debug log
    //SerialPC.print("angle: ");
    //SerialPC.print(angle);
    //SerialPC.print(" ball angle: ");
    //SerialPC.print(ball.ANGLE);
    //SerialPC.print(" distance: ");
    //SerialPC.print(ball.DISTANCE);
    //SerialPC.print(" moveAngle: ");
    //SerialPC.print(moveAngle);
    //SerialPC.print(" speed: ");
    //SerialPC.print(speed);
    //SerialPC.println();

    // Kick
    kick();

    // check button
    if (!digitalRead(Pause)) {
      gameFlag = false;
      setMotor(0, FL_FWD, FL_REV);
      setMotor(0, BL_FWD, BL_REV);
      setMotor(0, BR_FWD, BR_REV);
      setMotor(0, FR_FWD, FR_REV);
      for (int i = 0; i < 8; i++) {
        digitalWrite(LED[i], LOW);
      }
      errI = 0.0;
      prevErr = 0.0;
      break;
    }

    // move
    // read line angle
    LineAngle = getLine();
    /*
    while (!(LineAngle == -1.0)) {
      moveAngle = LineAngle;
      speed = 0.8;
      move_motor(speed, moveAngle, angle, tarAngle);
      LineAngle = getLine();
    }
    */
    move_motor(speed, moveAngle, angle, tarAngle);
  }

}

void writeEEPROM(int addr, byte data) {
  Wire2.beginTransmission(EEPROM_ADDR);
  Wire2.write((int)(addr >> 8));
  Wire2.write((int)(addr & 0xFF));
  Wire2.write(data);
  Wire2.endTransmission();
  delay(5);
}

byte readEEPROM(int addr) {
  Wire2.beginTransmission(EEPROM_ADDR);
  Wire2.write((int)(addr >> 8));
  Wire2.write((int)(addr & 0xFF));
  Wire2.endTransmission();

  Wire2.requestFrom(EEPROM_ADDR, 1);
  if (Wire2.available()) {
    return Wire2.read();
  }
  return 0;
}

void kick() {
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

double getAngle() {
  sensors_event_t ev;
  bno.getEvent(&ev);

  double heading = ev.orientation.x;
  return heading;
}

void setMotor(double power, int pinF, int pinR) {
  power = constrain(power, -1.0f, 1.0f);

  int pwm = fabs(power) * PWM_MAX;

  //SerialPC.println(pwm);

  if (power >= 0) {
    analogWrite(pinF, pwm);
    analogWrite(pinR, 0);
  } else {
    analogWrite(pinF, 0);
    analogWrite(pinR, pwm);
  }
}

double culcMoveAngle(double ballAngle) {
  double centered = ballAngle;
  if (centered > 180.0f) centered -= 360.0f;
  double Ang = centered * 1.5f;

  while (Ang < 0.0f) Ang += 360.0f;
  while (Ang >= 360.0f) Ang -= 360.0f;
  return Ang;
}

Ball getBall() {
  double X = 0.0, Y = 0.0;
  int Si[8];
  int min_val = analogRead(BALL[0]);

  for (int i = 0; i < 8; i++) {
    Si[i] = analogRead(BALL[i]);
    if (Si[i] < min_val) {
      min_val = Si[i];
    }

    double rad = radians(BALLANGLE[i]);
    X += Si[i] * cos(rad);
    Y += Si[i] * sin(rad);
  }

  Ball ballinfo;

  double ballAngle = atan2(Y, X) * 180.0 / PI;
  ballAngle += 180.0; // add offset to reverse
  if (ballAngle < 0) {
    ballAngle += 360.0;
  }

  while (ballAngle >= 360.0f) ballAngle -= 360.0f;
  while (ballAngle < 0.0f) ballAngle += 360.0f;

  ballinfo.ANGLE = ballAngle;

  int ledIndex = (int)round(ballAngle / 45.0) % 8;

  for (int i = 0; i < 8; i++) {
    if (i == ledIndex) {
      digitalWrite(LED[i], HIGH);
    } else {
      digitalWrite(LED[i], LOW);
    }
  }

  ballinfo.DISTANCE = min_val;

  return ballinfo;
}

double getLine() {
  double A = 0.0;
  SerialLine.write(LINE_INFO);
  //SerialPC.println("getLine now");

  byte buffer[4];
  SerialLine.readBytes(buffer, 4);
  /*
  SerialPC.print(buffer[0]);
  SerialPC.print(" ");
  SerialPC.print(buffer[1]);
  SerialPC.print(" ");
  SerialPC.print(buffer[2]);
  SerialPC.print(" ");
  SerialPC.println(buffer[3]);
  */
  memcpy(&A, buffer, 4);

  return A;
}

// MARK: move motor

void move_motor(double speed, double moveDeg, double heading, double tarHeading) {
  heading = getAngle();

  SerialPC.print(" speed "); 
  SerialPC.print(speed);
  SerialPC.print(" moveDeg ");
  SerialPC.print(moveDeg);
  SerialPC.print(" heading ");
  SerialPC.print(heading);
  SerialPC.print(" tarHeading ");
  SerialPC.println(tarHeading);
  //P計算
  double err = tarHeading - heading;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;

  //D計算
  unsigned long now = millis();
  double dt = (now - prevTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.000001f;
  double D = (err - prevErr) / dt;
  prevTime = now;
  prevErr = err;

  //PD計算
  double PID = Kp * err + Kd * D;
  PID = constrain(PID, -0.5f, 0.5f);

  double v_fr = /*-cos(radians(moveDeg + 45.0)) */ + PID;
  double v_br = /*-cos(radians(moveDeg - 45.0)) */ + PID;
  double v_bl = /*cos(radians(moveDeg + 45.0)) */ + PID;
  double v_fl = /*cos(radians(moveDeg - 45.0)) */ + PID;
  //正規化
  v_fr = constrain(v_fr, -1.0f, 1.0f);
  v_br = constrain(v_br, -1.0f, 1.0f);
  v_bl = constrain(v_bl, -1.0f, 1.0f);
  v_fl = constrain(v_fl, -1.0f, 1.0f);

  SerialPC.printf(">v_fr:");
  SerialPC.println(v_fr);
  SerialPC.printf(">v_br:");
  SerialPC.println(v_br);
  SerialPC.printf(">v_bl:");
  SerialPC.println(v_bl);
  SerialPC.printf(">v_fl:");
  SerialPC.println(v_fl);
  /*
  setMotor(v_fl, FL_FWD, FL_REV);
  setMotor(v_fr, FR_FWD, FR_REV);
  setMotor(v_bl, BL_FWD, BL_REV);
  setMotor(v_br, BR_FWD, BR_REV);
  */

  //setmotorを介さずに直接PWM出力
  if (v_fr >= 0) {
    analogWrite(FL_FWD, (int)(v_fr * PWM_MAX));
    analogWrite(FL_REV, 0);
  } else {
    analogWrite(FL_FWD, 0);
    analogWrite(FL_REV, (int)(fabs(v_fr) * PWM_MAX));
  }
  if (v_fr >= 0) {
    analogWrite(FR_FWD, (int)(v_fr * PWM_MAX));
    analogWrite(FR_REV, 0);
  } else {
    analogWrite(FR_FWD, 0);
    analogWrite(FR_REV, (int)(fabs(v_fr) * PWM_MAX));
  }
  if (v_bl >= 0) {
    analogWrite(BL_FWD, (int)(v_bl * PWM_MAX));
    analogWrite(BL_REV, 0);
  } else {
    analogWrite(BL_FWD, 0);
    analogWrite(BL_REV, (int)(fabs(v_bl) * PWM_MAX));
  }
  if (v_br >= 0) {
    analogWrite(BR_FWD, (int)(v_br * PWM_MAX));
    analogWrite(BR_REV, 0);
  } else {
    analogWrite(BR_FWD, 0);
    analogWrite(BR_REV, (int)(fabs(v_br) * PWM_MAX));
  }
}

void move(double speed, double moveDeg, double heading, double targetHeading) {
  double rad = moveDeg * M_PI / 180.0;
  double vx = speed * cos(rad);
  double vy = speed * sin(rad);

  double err = targetHeading - heading;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;

  double omega = Kp * err;
  omega = constrain(omega, -1.0f, 1.0f);

  // --- 4輪オムニ逆運動学基本式 ---
  double v_fl =  vx + vy + omega;
  double v_fr = -vx + vy + omega;
  double v_bl =  vx - vy + omega;
  double v_br = -vx - vy + omega;

  double max_v = max(max(fabs(v_fl), fabs(v_fr)), max(fabs(v_bl), fabs(v_br)));
  if (max_v > 1.0f) {
    v_fl /= max_v;
    v_fr /= max_v;
    v_bl /= max_v;
    v_br /= max_v;
  }

  setMotor(v_fl, FL_FWD, FL_REV);
  setMotor(v_fr, FR_FWD, FR_REV);
  setMotor(v_bl, BL_FWD, BL_REV);
  setMotor(v_br, BR_FWD, BR_REV);
}

void motorTest () {
  for (int i = 0; i < 256; i++) {
    analogWrite(FL_FWD, i);
    analogWrite(FL_REV, 0);
    analogWrite(FR_FWD, i);
    analogWrite(FR_REV, 0);
    analogWrite(BL_FWD, i);
    analogWrite(BL_REV, 0);
    analogWrite(BR_FWD, i);
    analogWrite(BR_REV, 0);
    delay(10);
  }
  analogWrite(FL_FWD, 0);
  analogWrite(FL_REV, 0);
  analogWrite(FR_FWD, 0);
  analogWrite(FR_REV, 0);
  analogWrite(BL_FWD, 0);
  analogWrite(BL_REV, 0);
  analogWrite(BR_FWD, 0);
  analogWrite(BR_REV, 0);
  for (int i = 0; i < 256; i++) {
    analogWrite(FL_FWD, 0);
    analogWrite(FL_REV, i);
    analogWrite(FR_FWD, 0);
    analogWrite(FR_REV, i);
    analogWrite(BL_FWD, 0);
    analogWrite(BL_REV, i);
    analogWrite(BR_FWD, 0);
    analogWrite(BR_REV, i);
    delay(10);
  }
  analogWrite(FL_FWD, 0);
  analogWrite(FL_REV, 0);
  analogWrite(FR_FWD, 0);
  analogWrite(FR_REV, 0);
  analogWrite(BL_FWD, 0);
  analogWrite(BL_REV, 0);
  analogWrite(BR_FWD, 0);
  analogWrite(BR_REV, 0);
}