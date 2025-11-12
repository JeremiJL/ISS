#include "TRSensors.h"
#include <stdio.h>

// Pins
#define RIGHT_MOTOR_FWD A0
#define RIGHT_MOTOR_REV A1
#define RIGHT_MOTOR_ENABLE 5

#define LEFT_MOTOR_FWD A3
#define LEFT_MOTOR_REV A2
#define LEFT_MOTOR_ENABLE 6

// Speed
#define DEFAULT_SPEED 100
#define ROTATION_SPEED 50
#define MOTOR_SWITCH_DELAY 100

// Sensors
const unsigned int CENTER_OF_THE_LINE = 200;
constexpr int NUM_SENSORS = 5;
unsigned int sensorValues[NUM_SENSORS];
TRSensors trs = TRSensors();


// Sensor calibration
void calibrateSensors(TRSensors trs){
  Serial.println("Calibration started");
  for (int i = 0; i < 500; i++) {
    trs.calibrate();
  }
  Serial.println("Calibration ended");

  Serial.println("Min values:");
  for (int i = 0; i < 5; i++ ) {
    Serial.print(trs.calibratedMin[i]);
    Serial.print(", ");
  }
  Serial.println();

  Serial.println("Max values:");
  for (int i = 0; i < 5; i++ ) {
    Serial.print(trs.calibratedMax[i]);
    Serial.print(", ");
  }
  Serial.println();
}

// Motor configuration
class Motor {
    int pinFwd;
    int pinRev;
    int pinPow;

    public:
        init() {
          pinMode(pinFwd, OUTPUT);
          pinMode(pinRev, OUTPUT);
          pinMode(pinPow, OUTPUT);
        }
        set_speed(int speed) {
          speed = constrain(speed, 0, 255);
          analogWrite(pinPow, speed);
        }
        set_direction(bool forward) {
          if (forward) {
              digitalWrite(this->pinRev, LOW);
              delay(MOTOR_SWITCH_DELAY);
              digitalWrite(this->pinFwd, HIGH);
          } else {
              digitalWrite(this->pinFwd, LOW);
              delay(MOTOR_SWITCH_DELAY);
              digitalWrite(this->pinRev, HIGH);
          }
        }
        stop(){
          set_speed(0);
        }
        Motor(int pinFwd, int pinRev, int pinPow) {
          this->pinFwd = pinFwd;
          this->pinRev = pinRev;
          this->pinPow = pinPow;
        }
};

// Motor's global declarations
Motor right_motor(RIGHT_MOTOR_FWD,RIGHT_MOTOR_REV,RIGHT_MOTOR_ENABLE);
Motor left_motor(LEFT_MOTOR_FWD,LEFT_MOTOR_REV,LEFT_MOTOR_ENABLE);

// Movements
void stop() {
  right_motor.stop();
  left_motor.stop();
}

void move_forward() {
    right_motor.set_direction(true);
    left_motor.set_direction(true);

    right_motor.set_speed(DEFAULT_SPEED);
    left_motor.set_speed(DEFAULT_SPEED);
}

void calibrateToRight() {
    right_motor.set_direction(false);
    left_motor.set_direction(true);

    right_motor.set_speed(ROTATION_SPEED);
    left_motor.set_speed(ROTATION_SPEED);

    while (trs.readLine(sensorValues) < CENTER_OF_THE_LINE) {}

    stop();
}

void calibrateToLeft() {
    right_motor.set_direction(true);
    left_motor.set_direction(false);

    right_motor.set_speed(ROTATION_SPEED);
    left_motor.set_speed(ROTATION_SPEED);

    while (trs.readLine(sensorValues) > CENTER_OF_THE_LINE) {}

    stop();
}

void calibrate() {
    unsigned int offset = trs.readLine(sensorValues);
    if (offset < CENTER_OF_THE_LINE)
      calibrateToRight();
    if (offset > CENTER_OF_THE_LINE)
      calibrateToLeft();
}

void setup() {
  // Monitor output
  Serial.begin(9600);

  // Initialzie motors
  left_motor.init();
  right_motor.init();

  // Calibrate sensors
  calibrateSensors(trs);
}

void loop() {
    // Move forward for at least 100 ms
    move_forward();
    delay(100);
    // After that, calibrate to match the line
    calibrate();
    // Then move forward accoring to the line again...
}
