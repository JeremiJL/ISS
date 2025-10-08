#include <string.h>

// Protocol ;
// Move right by x cm : move-right x
String MOVE_RIGHT = "move-right";
// Move left by x cm : move-left x
String MOVE_LEFT = "move-left";
// Move foreward by x cm : move-forward x
String MOVE_FOREWARD = "move-foreward";
// Move backwards by x cm : move-bacward x
String MOVE_BACKWARDS = "move-backwards";
// Rotate in left direction by y angle : rotate-left y
String ROTATE_LEFT = "rotate-left";
// Rotate in right direction by y angle : rotate-right y
String ROTATE_RIGHT = "rotate-right";


#define RIGHT_MOTOR_IN1 A0
#define RIGHT_MOTOR_IN2 A1
#define RIGHT_MOTOR_ENABLE 5

#define LEFT_MOTOR_IN1 A2
#define LEFT_MOTOR_IN2 A3
#define LEFT_MOTOR_ENABLE 6

#define MOTOR_SWITCH_DELAY 20

#define ENCODER_PIN_R 2
#define ENCODER_PIN_L 3

volatile int counter_r = 0;
volatile int counter_l = 0;

float const signals_by_cm_ratio = 0.857;

void set_motor_direction(bool right, bool backwards) {
    if (right) {
        if (backwards) {
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
        } else {
            digitalWrite(RIGHT_MOTOR_IN2, LOW);
            delay(MOTOR_SWITCH_DELAY);
            digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        }
    } else {
        if (backwards) {
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        delay(MOTOR_SWITCH_DELAY);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
        } else {
            digitalWrite(RIGHT_MOTOR_IN2, LOW);
            delay(MOTOR_SWITCH_DELAY);
            digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        }
    }   
}

void increment_right_counter() {
  counter_r++;
}

void increment_left_counter() {
  counter_l++;
}

void printState(String order) {
    Serial.print("Right counter: ");
    Serial.print(counter_r);
    Serial.print("\n");

    Serial.print("Left counter: ");
    Serial.print(counter_l);
    Serial.print("\n");

    Serial.print("Order : ");
    Serial.print(order);
    Serial.print("\n");

}

void move(int distance, bool backwards, bool, right) {
    set_motor_direction(true, backwards);
    int ticks = (int)(signals_by_cm_ratio * distance);
    int goal_ticks = counter_r + ticks;
    analogWrite(RIGHT_MOTOR_ENABLE, constrain(100, 0, 255));
    while(counter_r < goal_ticks) {}
    analogWrite(RIGHT_MOTOR_ENABLE, constrain(0, 0, 255));
}

void moveRight(int distance, bool backwards) {
    move(distance, backwards, right)
}
void moveLeft(int distance, bool backwards){

}
void rotateRight(int angle) {

}
void rotateLeft(int angle) {
    
}

void processOrders(String order) {
    if (order.startsWith(MOVE_RIGHT)){
        int distance = order.substring(MOVE_RIGHT.length(), order.length()).toInt();
        moveRight(distance, false);
        Serial.print("supposed to move right");
    } if (order.startsWith(MOVE_LEFT)){
        int distance = order.substring(MOVE_LEFT.length(), order.length()).toInt();
        moveLeft(distance, false);
    } if (order.startsWith(ROTATE_LEFT)){
        int angle = order.substring(ROTATE_LEFT.length(), order.length()).toInt();
        rotateLeft(angle);
    } if (order.startsWith(ROTATE_RIGHT)){
        int angle = order.substring(ROTATE_RIGHT.length(), order.length()).toInt();
        rotateRight(angle);
    } if (order.startsWith(MOVE_FOREWARD)){
        int distance = order.substring(MOVE_FOREWARD.length(), order.length()).toInt();
        // it may not work (if it's done sequentially)
        moveRight(distance, false);
        moveLeft(distance, false);
    } if (order.startsWith(MOVE_BACKWARDS)){
        int distance = order.substring(MOVE_BACKWARDS.length(), order.length()).toInt();
        // it may not work (if it's done sequentially)
        moveRight(distance, true);
        moveLeft(distance, true);
    } else {
        Serial.print("nothing matched");
    }
}


void setup() {
    // Start serial
    Serial.begin(9600);
    // Coonfigure wheels state monitoring
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), increment_right_counter, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), increment_left_counter, RISING);
    // Configure wheels' motors
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
}

void loop() {
    // Read from
    String order = Serial.readStringUntil('\n');
    // Process orders
    processOrders(order);
    // Display state
    printState(order);
}

// Notes :

// X = 18 [signal rises]
// Y = 21 [cm]
// X/Y = 0.857 [signal/cm]


