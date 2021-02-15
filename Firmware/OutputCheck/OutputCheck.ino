// Motor Pins
// Motor 1 | PA2 PA1 | 15 14
// Motor 2 | PA3 PC3 | 16 13
// Motor 3 | PC2 PC1 | 12 11
// Motor 4 | PC0 PB0 | 10 9
// Motor 5 | PB1 PB2 | 8 7
// Motor 6 | PB3 PB4 | 6 5

volatile uint8_t current_motor;
uint8_t forwards[] = {15,16,12,10,7,5};
uint8_t backwards[] = {14,13,11,9,8,6};

void setup() {
  // Set all motor pins as outputs
  for (int i=0; i < 6; i++){
    pinMode(backwards[i],OUTPUT);
    pinMode(forwards[i],OUTPUT);
  }

}

void loop() {
  disable_motors();
  delay(1000);
  enable_motors(1);
  delay(1000);
  disable_motors();
  delay(1000);
  enable_motors(1);
  delay(1000);
}

// Pulls all the motor outputs low to turn them off
void disable_motors(){
  for (int i=0; i < 6; i++){
    digitalWrite(forwards[i],0);
    digitalWrite(backwards[i],0);
  }
}

void enable_motors(bool dir){
  if (dir){
    for (int i=0; i < 6; i++){
    digitalWrite(forwards[i],1);
    digitalWrite(backwards[i],0);
  }
  } else {
    for (int i=0; i < 6; i++){
    digitalWrite(forwards[i],0);
    digitalWrite(backwards[i],1);
  }
  }
}