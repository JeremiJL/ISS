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

#define ENCODER_PIN_R 2
#define ENCODER_PIN_L 3

int counter_r = 0;
int counter_l = 0;

void increment_right_counter() {
  counter_r++;
}

void increment_left_counter() {
  counter_l++;
}

void printState() {
    Serial.print("Right counter: ");
    Serial.print(counter_r);
    Serial.print("\n");

    Serial.print("Left counter: ");
    Serial.print(counter_l);
    Serial.print("\n");
}

void moveRight(int distance, bool backwards) {
    
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
        ;
    }
}


void setup() {
    // Start serial
    Serial.begin(9600);
    // Coonfigure wheels state monitoring
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), increment_right_counter, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), increment_left_counter, RISING);
}

void loop() {
    // Read from
    String order = Serial.readStringUntil('\n');
    // Process orders
    processOrders(order);
    // Display state
    printState();
}

// 


