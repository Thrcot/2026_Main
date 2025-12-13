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
float BALLANGLE[8] = {0, 45, 90, 135, 180, 225, 270, 315};

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
const int PWM_MAX = 255;
bool gameFlag = false;
unsigned long lastCatchTime = 0;
bool catching = false;

int menu = 0;
const unsigned char memuDisplay[] = {};

#define SPEED 0.6

#define LINE_INIT 0xAD
#define LINE_INFO 0xAE
#define LINE_OK 0xAF

struct Ball {
  float ANGLE;
  float DISTANCE;
};
const float BALL_THRESHOLD = 280.0;

// PID
float Kp = 0.008;
float Ki = 0.00025;
float Kd = 0.002;

float errI = 0.0;
float prevErr = 0.0;

// define functions
void writeEEPROM(int addr, byte data);
byte readEEPROM(int addr);
void kick();
float culcMoveAngle(float ballAngle);
float getAngle();
float getLine();
Ball getBall();
void setMotor(float power, int pinF, int pibR);
void move_motor(float speed, float moveDeg, float heading, float tarHeading);
void move(float speed, float moveDeg, float heading, float targetHeading);
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

void loop() {

  //motorTest();

  if (!digitalRead(Enter)){
    gameFlag = true;
  }
  while (gameFlag) {
    float angle = 0.0;
    float moveAngle = 0.0;
    float speed = 0.0;
    float tarAngle = 0.0;
    float LineAngle = 0.0;
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
  if (analogRead(ADCCatch) < 200) {
    if (!catching) {
      lastCatchTime = millis();
      catching = true;
    }
    if (millis() - lastCatchTime > 10) {
      digitalWrite(Kick, HIGH);
      delay(100);
      digitalWrite(Kick, LOW);
      catching = false;
    }else{
      digitalWrite(Kick, LOW);
    }
  } else {
    catching = false;
    digitalWrite(Kick, LOW);
  }
}

float getAngle() {
  sensors_event_t ev;
  bno.getEvent(&ev);

  float heading = ev.orientation.x;
  return heading;
}

void setMotor(float power, int pinF, int pinR) {
  if (power > 1.0f)  power = 1.0f;
  if (power < -1.0f) power = -1.0f;

  int pwm = fabs(power) * PWM_MAX;

  SerialPC.println(pwm);

  if (power >= 0) {
    analogWrite(pinF, pwm);
    analogWrite(pinR, 0);
  } else {
    analogWrite(pinF, 0);
    analogWrite(pinR, pwm);
  }
}

float culcMoveAngle(float ballAngle) {
  float centered = ballAngle;
  if (centered > 180.0f) centered -= 360.0f;
  float Ang = centered * 1.5f;

  while (Ang < 0.0f) Ang += 360.0f;
  while (Ang >= 360.0f) Ang -= 360.0f;
  return Ang;
}

Ball getBall() {
  float X = 0.0, Y = 0.0;
  int Si[8];
  int min_val = analogRead(BALL[0]);

  for (int i = 0; i < 8; i++) {
    Si[i] = analogRead(BALL[i]);
    if (Si[i] < min_val) {
      min_val = Si[i];
    }

    float rad = radians(BALLANGLE[i]);
    X += Si[i] * cos(rad);
    Y += Si[i] * sin(rad);
  }

  Ball ballinfo;

  float ballAngle = atan2(Y, X) * 180.0 / PI;
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

float getLine() {
  SerialLine.println(LINE_INFO, HEX);
  ;
}

void move_motor(float speed, float moveDeg, float heading, float tarHeading) {
  float err = tarHeading - heading;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;

  float omega = Kp * err;
  omega = constrain(omega, -0.3f, 0.3f);

  float v_fr = sin((float)radians(moveDeg - 45.0)) * speed + omega;
  float v_br = sin((float)radians(moveDeg - 135.0)) * speed + omega;
  float v_bl = sin((float)radians(moveDeg - 225.0)) * speed + omega;
  float v_fl = sin((float)radians(moveDeg - 315.0)) * speed + omega;

  setMotor(v_fl, FL_FWD, FL_REV);
  setMotor(v_fr, FR_FWD, FR_REV);
  setMotor(v_bl, BL_FWD, BL_REV);
  setMotor(v_br, BR_FWD, BR_REV);
}

void move(float speed, float moveDeg, float heading, float targetHeading) {
  float rad = moveDeg * M_PI / 180.0;
  float vx = speed * cos(rad);
  float vy = speed * sin(rad);

  float err = targetHeading - heading;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;

  float omega = Kp * err;
  omega = constrain(omega, -1.0f, 1.0f);

  // --- 4輪オムニ逆運動学基本式 ---
  float v_fl =  vx + vy + omega;
  float v_fr = -vx + vy + omega;
  float v_bl =  vx - vy + omega;
  float v_br = -vx - vy + omega;

  float max_v = max(max(fabs(v_fl), fabs(v_fr)), max(fabs(v_bl), fabs(v_br)));
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