#include <string.h>

// Protocol ;
// Move right by x cm : move-right x
String MOVE_RIGHT = "move-right";
// Move left by x cm : move-left x
String MOVE_LEFT = "move-left";
// Move foreward by x cm : move-forwards x
String MOVE_FOREWARD = "move-forwards";
// Move backwards by x cm : move-backwards x
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
float const angle_to_distance_ratio = 0.5;

String const path = "move-forwards 10\nrotate-right 90\nmove-backwards 4\nrotate-left 45\nmove-right 6\n";

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

void set_motor_speed(int motor_pin, int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(motor_pin, speed);
}

void move(int distance_in_cm, bool backwards, bool right) {
    int ticks = (int)(signals_by_cm_ratio * distance_in_cm);
    set_motor_direction(true, backwards);

    if (right) {
        int goal_ticks = counter_r + ticks;
        set_motor_speed(RIGHT_MOTOR_ENABLE, 255);
        while(counter_r < goal_ticks) {
        }
        set_motor_speed(RIGHT_MOTOR_ENABLE, 0);
    } else {
        int goal_ticks = counter_l + ticks;
        set_motor_speed(LEFT_MOTOR_ENABLE, 255);
        while(counter_l < goal_ticks) {
        }
        set_motor_speed(LEFT_MOTOR_ENABLE, 0);
    }
}

void moveRight(int distance_in_cm, bool backwards) {
    move(distance_in_cm, backwards, true);
}
void moveLeft(int distance_in_cm, bool backwards){
    move(distance_in_cm, backwards, false);
}

void rotateRight(int angle) {
    int distance_in_cm = angle * angle_to_distance_ratio;
    moveRight(distance_in_cm, true);
    moveLeft(distance_in_cm, false);
}
void rotateLeft(int angle) {
    int distance_in_cm = angle * angle_to_distance_ratio;
    moveRight(distance_in_cm, false);
    moveLeft(distance_in_cm, true);
}

void processOrders(String order) {
    if (order.startsWith(MOVE_RIGHT)){
        int distance_in_cm = order.substring(MOVE_RIGHT.length(), order.length()).toInt();
        moveRight(distance_in_cm, false);
    } if (order.startsWith(MOVE_LEFT)){
        int distance_in_cm = order.substring(MOVE_LEFT.length(), order.length()).toInt();
        moveLeft(distance_in_cm, false);
    } if (order.startsWith(ROTATE_LEFT)){
        int angle = order.substring(ROTATE_LEFT.length(), order.length()).toInt();
        rotateLeft(angle);
    } if (order.startsWith(ROTATE_RIGHT)){
        int angle = order.substring(ROTATE_RIGHT.length(), order.length()).toInt();
        rotateRight(angle);
    } if (order.startsWith(MOVE_FOREWARD)){
        int distance_in_cm = order.substring(MOVE_FOREWARD.length(), order.length()).toInt();
        // it may not work (if it's done sequentially)
        moveRight(distance_in_cm, false);
        moveLeft(distance_in_cm, false);
    } if (order.startsWith(MOVE_BACKWARDS)){
        int distance_in_cm = order.substring(MOVE_BACKWARDS.length(), order.length()).toInt();
        // it may not work (if it's done sequentially)
        moveRight(distance_in_cm, true);
        moveLeft(distance_in_cm, true);
    } else {
        Serial.print("nothing matched\n");
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
    // 
    // set_motor_speed(RIGHT_MOTOR_ENABLE, 255);

}

// Notes :

// X = 18 [signal rises]
// Y = 21 [cm]
// X/Y = 0.857 [signal/cm]


