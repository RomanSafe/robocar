#include <Servo.h>
// подключение библиотеки для HC SR04
#include "Ultrasonic.h"

// константы для выводов
#define PIN_TRIG 2
#define PIN_ECHO 3
#define PIN_PE 4

#define IN_1  6    // L298N in 1 right motor 
#define IN_2  7    // L298N in 2 right motor 
#define IN_3  8    // L298N in 3 left motor
#define IN_4  9    // L298N in 4 left motor

#define AHEAD 90   // angle of servo position
#define LEFT 45
#define RIGHT 135

#define CHECKING_TIME_DOWNTIME 15000
#define QUANTITY_OF_MEASURE 3

//#define CHECKING_TIME_VOLTAGE 60000
//#define MIN_VOLTAGE 3
#define MEASURE_OF_VOLTAGE A5
#define KEY_OF_MOSFET 5

Servo myservo;  // create servo object to control a servo
Ultrasonic ultrasonic(PIN_TRIG, PIN_ECHO);// создание объекта Ultrasonic

// переменные для хранения измеренного расстояния
int dist_left1 = 0;
int dist_right1 = 0;
int dist_ahead1 = 0;
int dist_left2 = 0;
int dist_right2 = 0;
int dist_ahead2 = 0;

unsigned long last_checking_downtime = 0;

//unsigned long last_checking_voltage = 60000;

// the all functions
void mil_delay(unsigned long * delay_time);
void goAhead();
void goBack();
void goRight();
void goLeft();
void stopRobot();
void observe();
int measure();
int lookAhead();
int lookLeft();
int lookRight();
void checkDowntime();
void exitFromDowntime();
//void checkVoltage();


void setup() {
  myservo.attach(10);  // attaches the servo on pin (?) to the servo object

  //  Serial.begin(9600);

  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  pinMode(PIN_PE, OUTPUT);// назначить P-E как OUTPUT
  digitalWrite(PIN_PE, LOW);

  pinMode(KEY_OF_MOSFET, OUTPUT);
  digitalWrite(KEY_OF_MOSFET, LOW);
}


void loop() {
  if (lookAhead() > 35) {
    goAhead();
  } else {
    stopRobot();
    observe();
  }
  checkDowntime();
  //  checkVoltage();
}

//void checkVoltage() {
//  if (millis() - last_checking_voltage >= CHECKING_TIME_VOLTAGE) {
//    float voltage = (float)(analogRead(0) * 5.0) / 1024;
//    if (voltage <= MIN_VOLTAGE) {
//      digitalWrite(KEY_OF_MOSFET, LOW);
//    }
//    else {
//      digitalWrite(KEY_OF_MOSFET, HIGH);
//    }
//    Serial.println(voltage);
//    last_checking_voltage = millis();
//  }
//
//}

void checkDowntime() {
  if (millis() - last_checking_downtime >= CHECKING_TIME_DOWNTIME) {
    dist_left2 = lookLeft();
    int left_mod = max(dist_left1, dist_left2) - min(dist_left1, dist_left2);
    //    int left_mod = dist_left1 - dist_left2;
    dist_ahead2 = lookAhead();
    int ahead_mod = max(dist_ahead1, dist_ahead2) - min(dist_ahead1, dist_ahead2);
    //    int ahead_mod = dist_ahead1 - dist_ahead2;
    dist_right2 = lookRight();
    int right_mod = max(dist_right1, dist_right2) - min(dist_right1, dist_right2);
    //    int right_mod = dist_right1 - dist_right2;
    if (left_mod <= 15 or ahead_mod <= 15 or right_mod <= 15) {
      digitalWrite(PIN_PE, HIGH);
      exitFromDowntime();
    }
    dist_left1 = dist_left2;
    dist_ahead1 = dist_ahead2;
    dist_right1 = dist_right2;
    last_checking_downtime = millis();
  }
}

void exitFromDowntime() {
  do {
    goBack();
  } while (lookAhead() < 30);
  stopRobot();
  unsigned long term = 300;
  mil_delay(&term);
  observe();
}


int measure() {
  int distance = 0;
  for (int i = 0; i < QUANTITY_OF_MEASURE; i++) {
    distance += ultrasonic.read();
  }
  return (distance / QUANTITY_OF_MEASURE);
}


void observe() {
  // Serial.println(F("observe"));
  if (lookLeft() < lookRight()) {
    do {
      goRight();
    } while (lookLeft() < 35 && lookAhead() < 35 && lookRight() < 35);
  } else if (lookLeft() >= lookRight()) {
    do {
      goLeft();
    } while (lookLeft() < 35 && lookAhead() < 35 && lookRight() < 35);
  }
  digitalWrite(PIN_PE, LOW);
}


int lookAhead() {
  myservo.write(AHEAD);
  unsigned long term = 50;
  mil_delay(&term);
  int dist_ahead = measure();// control of distance
  // Serial.println(F("dist_ahead"));
  // Serial.println(dist_ahead);
  return dist_ahead;
}


int lookLeft() {
  myservo.write(LEFT);// check the left side
  unsigned long term = 50;
  mil_delay(&term);
  int dist_left = measure();
  // Serial.println(F("dist_left"));
  // Serial.println(dist_left);
  return dist_left;
}


int lookRight() {
  myservo.write(RIGHT);// check the right side
  unsigned long term = 50;
  mil_delay(&term);
  int dist_right = measure();
  // Serial.println(F("dist_right"));
  // Serial.println(dist_right);
  return dist_right;
}


void goAhead() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);

  // Serial.println(F("go ahead"));
}


void goBack() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);

  // Serial.println(F("go back"));
}


void goRight() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);

  // Serial.println(F("go right"));
}


void goLeft() {
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);

  // Serial.println(F("go left"));
}


void stopRobot() {
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);

  // Serial.println(F("stop"));
}


//settings and call of mil_delay()
//unsigned long term = 1000;
//mil_delay(&term);

void mil_delay(unsigned long * delay_time) {
  unsigned long counter = millis();
  // Serial.println(F("Before while"));
  while (millis() - counter < *delay_time) {
  }
  // Serial.println(F("After while"));
}
