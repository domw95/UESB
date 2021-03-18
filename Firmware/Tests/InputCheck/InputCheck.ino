/*
    Reads the inputs from the switch and RC-IN and sends them to outputs
    Switches 1 to 4 are mirrored to motor outputs 1 to 4 (4 not curently enabled)
    Inputs RC 1 and 2 are mirrored to motor outputs 5 and 6
*/

// Motor Pins
// Motor 1 | PA2 PA1 | 15 14
// Motor 2 | PA3 PC3 | 16 13
// Motor 3 | PC2 PC1 | 12 11
// Motor 4 | PC0 PB0 | 10 9
// Motor 5 | PB1 PB2 | 8 7
// Motor 6 | PB3 PB4 | 6 5

uint8_t forwards[] = {15,16,12,10,7,5};
uint8_t backwards[] = {14,13,11,9,8,6};
uint8_t inputs[] = {0,1,2,17,4,3};

void setup() {
  // Set all motor pins as outputs
  for (int i=0; i < 6; i++){
    pinMode(backwards[i], OUTPUT);
    pinMode(forwards[i], OUTPUT);
  }
  // set inputs pins
  for (int i=0; i < 6; i++){
    pinMode(inputs[i], INPUT_PULLUP);
  }
  disable_motors();
}

void loop() {
  // read all inputs and write as ouputs
  for (int i=0; i<6; i++){
    digitalWrite(forwards[i],digitalRead(inputs[i]));
  }
  
}

// Pulls all the motor outputs low to turn them off
void disable_motors(){
  for (int i=0; i < 6; i++){
    digitalWrite(forwards[i],0);
    digitalWrite(backwards[i],0);
  }
}
