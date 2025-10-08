char incomingChar;
#define ENCODER_PIN_R 2
#define ENCODER_PIN_L 3

int counter_r = 0;
int counter_l = 0;


void increment_r() {
  counter_r++;
}

void increment_l() {
  counter_l++;
}

void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_R), increment_r, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_L), increment_l, RISING);


}

void interrupt() {
  Serial.print("Right counter: ");
  Serial.print(counter_r);
  Serial.print("\n");

  Serial.print("Left counter: ");
  Serial.print(counter_l);
  Serial.print("\n");
}

void loop() {

  if (Serial.available() > 0) {
    incomingChar = Serial.read();
    // Serial.print(incomingChar);
  }

  if (incomingChar == 'i') {
    Serial.println(incomingChar);
    interrupt();
  }

}




