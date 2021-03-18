#include "Servo.h"

bool sweep = false;

Servo servo1;
Servo servo2;

uint32_t prev;
uint16_t motor;

void setup() {
  Serial.begin(115200);
  servo1.attach(9);
  servo2.attach(10);
  motor = 1000;
  pinMode(3, INPUT_PULLUP);
}

void loop() {

  if (sweep){
    // Go through each motor
    uint16_t motor,pwm;
    for (motor=1000; motor < 2001; motor+=200){
      servo1.writeMicroseconds(motor);
      // cycle through pwm values
      for (pwm=1500; pwm<1900; pwm++){      
        servo2.writeMicroseconds(pwm);
        delay(1);
      }
      for (pwm=1900; pwm>1100; pwm--){      
        servo2.writeMicroseconds(pwm);
        delay(2);
      }
      for (pwm=1000; pwm<1500; pwm++){      
        servo2.writeMicroseconds(pwm);
        delay(2);
      }
    }
  } else{
    uint32_t now = millis();
    if (now - prev > 100){
      // get inputs from pot and buttons
      if (!digitalRead(3)){
        motor += 200;
        if (motor > 2000){
          motor = 1000;
        }
      }
      prev = now;
      uint16_t ain = analogRead(A0);
      uint16_t pwm = map(ain, 0, 1024, 1000, 2000);
      servo1.writeMicroseconds(motor);
      servo2.writeMicroseconds(pwm);
    }
    
  }
}
