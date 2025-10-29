#include "TRSensors.h"
#include <stdio.h>

#define RIGHT_MOTOR_FWD A0
#define RIGHT_MOTOR_REV A1
#define RIGHT_MOTOR_ENABLE 5

#define LEFT_MOTOR_FWD A3
#define LEFT_MOTOR_REV A2
#define LEFT_MOTOR_ENABLE 6

#define MOTOR_SWITCH_DELAY 100

// Sensors
constexpr int num_sensors = 5;
unsigned int sensorValues[num_sensors];


// Print customization
int put(char c, FILE *f) {
  Serial.write(c);
  return 0;
}
void fix_printf() {
  fdevopen(&put, 0);
}

// Robot control
void PID(){

}

// Sensor calibration
void calibrateSensors(TRSensors trs){
  Serial.println("Calibration started");
  for (int i = 0; i < 500; i++) {
    trs.calibrate();
  }
  Serial.println("Calibration ended");

  Serial.println("Min values:");
  for (int i = 0; i < 5; i++ ) {
    printf("%d ", trs.calibratedMin[i]);
  }
  Serial.println();
  
  Serial.println("Max values:");
  for (int i = 0; i < 5; i++ ) {
    printf("%d ", trs.calibratedMax[i]);
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
        stop();

        Motor(int pinFwd, int pinRev, int pinPow) {
          this->pinFwd = pinFwd;
          this->pinRev = pinRev;                 
          this->pinPow = pinPow;
        }
};

// Global declarations
Motor left(RIGHT_MOTOR_FWD,RIGHT_MOTOR_REV,RIGHT_MOTOR_ENABLE);
Motor right(LEFT_MOTOR_FWD,LEFT_MOTOR_REV,LEFT_MOTOR_ENABLE);
TRSensors trs = TRSensors();

void setup() {
  // Monitor output
  Serial.begin(9600);
  fix_printf();

  // Initialzie motors
  left.init();
  right.init();

  // Calibrate sensors
  calibrateSensors(trs);
}

void loop() {
    // process serial input
    // read the sensors and adjust the motor speeds (see TRSensorsExample.ino)
    // this loop should run around 10 times per second, make sure that no function stalls
}