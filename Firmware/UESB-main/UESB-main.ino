// Motor Pins
// Motor   | Portpin | GPIO  | Pin
// Motor 1 | PA2 PA1 | 15 14 | 18 17
// Motor 2 | PA3 PC3 | 16 13 | 19 15
// Motor 3 | PC2 PC1 | 12 11 | 14 13
// Motor 4 | PC0 PB0 | 10 9  | 12 11
// Motor 5 | PB1 PB2 | 8 7   | 10 9
// Motor 6 | PB3 PB4 | 6 5   | 8 7

// Input Pins
// SWITCH | Pin
// 1      | UPDI
// 2      | PA6
// 3      | PA5
// 4      | PA4

// RC Pins
// PB5
// PA7

// #define SERIALDEBUG

// Motor ouput port pins
#ifdef SERIALDEBUG
uint8_t mPortA = 0b1000;
#else
uint8_t mPortA = 0b1110;
#endif
uint8_t mPortB = 0b11111;
uint8_t mPortC = 0b1111;

// struct to handle servo inputs from rc pins
struct RCIN{
  bool state;     // whether the pin is high or low
  uint16_t risetime;
  uint16_t pulse;  // length of pulse in (us)
  bool updated;
};
volatile RCIN rcin[2];

// thresholds for pot mode on RC 1 in us
const uint16_t rcThresholds[] = {1100,1300,1500,1700,1900};
// motor speed selection
const uint16_t rcDeadband = 50;
const uint16_t rcMax = 399;

enum RCMODE{
  SWITCH,   // Increment/Decrement current motor with 2/3 way switch
  POT       // Select motor with a linear input (potentiometer)
};

enum SWITCHLEVEL{
  SL_LOW,
  SL_MED,
  SL_HIGH
};

struct RCSWITCHCTRL{
  bool init;
  SWITCHLEVEL level;      // Current level of RC input low>1250<med>1750<high
  uint8_t debounce_count;
} rc_switch_ctrl;

struct State{
  uint8_t motor;    // Current motor (1 to 6)
  bool dir;         // Current direction (1 = forwards, 0 = reverse)
  RCMODE rcmode;
};
volatile State state;

void setup() {
  // Serial debug
  #ifdef SERIALDEBUG
  Serial.swap(1);
  Serial.begin(115200);
  Serial.println("Starting");
  #endif
  // Set all motor pins as outputs
  PORTA.DIRSET = mPortA;
  PORTB.DIRSET = mPortB;
  PORTC.DIRSET = mPortC;

  // Set the switch inputs as pullups
  PORTA.PIN4CTRL = 0b00001000;
  PORTA.PIN5CTRL = 0b00001000;
  PORTA.PIN6CTRL = 0b00001000;

  // Set the RC inputs as pullup and pinchange interrupt
  PORTA.PIN7CTRL = 0b00001001;
  PORTB.PIN5CTRL = 0b00001001;

  // check the switch states to setup system
  state.motor = 1;
  check_switches();

  // Motor control timer
  TCA0.SPLIT.CTRLA=0; //disable TCA0 and set divider to 1
  TCA0.SPLIT.CTRLESET=TCA_SPLIT_CMD_RESET_gc|0x03; //set CMD to RESET to do a hard reset of TCA0 and enable for both halves.
  TCA0.SPLIT.CTRLD=0; //turn off split mode
  TCA0.SINGLE.CTRLA = 0b1001;    //enable TCA and set prescale to 16 (1MHz)
  TCA0.SINGLE.INTCTRL = 0b00010001;  // enable cmp0 and overflow interrupt
  TCA0.SINGLE.PER = 400;     // timer will overflow every 400us (2.5kHz) (makes mapping super easy)
  TCA0.SINGLE.CMP0BUF = 0;    // initial duty cycle is 0

  // RCIN timer
  TCB0.CTRLA = 0b00000101;   // enable timer with clk from TCA0 (1MHz)


  
}

void loop() {
  check_switches();
  check_motor_selection();
    // check motor speed input
  if(rcin[1].updated){
    // remove updated flag
    rcin[1].updated = false;
    #ifdef SERIALDEBUG
    // Serial.println(rcin[1].pulse);
    #endif
    // check direction and magnitude
    bool dir = rcin[1].pulse > 1500;
    if (dir != state.dir){
      disable_motor(state.motor);
    }
    state.dir = dir;
    int16_t mag = abs(rcin[1].pulse - 1500) - rcDeadband;
    if (mag < 0){
      mag = 0;
    } else if (mag > rcMax){
      mag = rcMax;
    }
    TCA0.SINGLE.CMP0BUF = mag;
  }
  
}

void check_motor_selection(){
  
  // Checks if the servo input on RC in 1 has updated and selects motor accordingly
  if (rcin[0].updated){
    // common variables
    rcin[0].updated = false;
    uint8_t motor;
    uint16_t pulse = rcin[0].pulse;
    // #ifdef SERIALDEBUG
    // Serial.println(pulse);
    // #endif

    // pot/linear control mode
    if (state.rcmode == POT){
      if (pulse > rcThresholds[4]){
        motor = 6;
      } else {
        for (uint8_t i=0; i<5; i++){
          // default motor to 5, as it cant be selected in for loop      
          if (pulse < rcThresholds[i]){
            motor = i + 1;
            break;
          }
        }
      }
    
    // switch control mode
    } else if (state.rcmode == SWITCH){
      // check if currently in debounce
      if (rc_switch_ctrl.debounce_count){
        // decrement and return
        rc_switch_ctrl.debounce_count--;
        return;
      }

      // get level      
      SWITCHLEVEL level;
      if (pulse < 1250){
        level = SL_LOW;
      } else if (pulse < 1750){
        level = SL_MED;
      } else {
        level = SL_HIGH;
      }

      // check if first call to this function
      if (!rc_switch_ctrl.init){        
        rc_switch_ctrl.init = true;
        // assign level and return
        rc_switch_ctrl.level = level;
        return;
      }

      // check if level has changed
      if (level != rc_switch_ctrl.level){
        // debounce for 5 cycles (about 0.1s)
        rc_switch_ctrl.debounce_count = 4;
        // check the switch that has occured
        SWITCHLEVEL prev_level = rc_switch_ctrl.level;
        
        if ((prev_level == SL_LOW && level == SL_HIGH) || (prev_level == SL_HIGH && level == SL_LOW)){
          // low to high or high to low transition (2 pos switch)
          motor = state.motor + 1;
        } else if (prev_level == SL_MED){
          // middle to h/l transition of 3 pos switch
          if (level == SL_LOW){
            // mid to low, decrement motor
            motor = state.motor - 1;
          } else if (level == SL_HIGH){
            // mid to high, increment motor
            motor = state.motor + 1;
          }
        }
        // update level
        rc_switch_ctrl.level = level;
        #ifdef SERIALDEBUG
        Serial.printf("%u to Level %u\n", prev_level, level);
        Serial.printf("%u to Motor %u\n", state.motor, motor);
        #endif
      }
    }
    // check if motor has changed
    if (motor != state.motor){
      disable_motor(state.motor);
    }
    // check motor wraparound
    if (motor == 0){
      motor = 6;
    } else if(motor > 6){
      motor = 1;
    }
    // update motor
    state.motor = motor;
    #ifdef SERIALDEBUG
    // Serial.printf("Motor %u\n",motor);
    #endif
  }
}

void check_switches(){
  // Read switches
  byte switch_state = PORTA.IN;

  // switch 2 determines RC input mode
  if (switch_state & (1 << 6)){
    state.rcmode = POT;
    #ifdef SERIALDEBUG
    // Serial.println("POT");
    #endif
  } else {
    state.rcmode = SWITCH;
    #ifdef SERIALDEBUG
    // Serial.println("SWI");
    #endif
  }

  // bool switch3 = switch_state & (1 << 5);
  // bool switch4 = switch_state & (1 << 4);
}

// Pulls all the motor outputs low to turn them off
void disable_motors(){
  PORTA.OUTSET = mPortA;
  PORTB.OUTSET = mPortB;
  PORTC.OUTSET = mPortC;
}

inline void disable_motor(uint8_t motor){
  switch(motor){
    case 1:
      #ifndef DEBUGSERIAL
      PORTA.OUTCLR = 0b110;
      #endif
      break;
    case 2:
      PORTA.OUTCLR = (1<<3);
      PORTC.OUTCLR = (1<<3);
      break;
    case 3:
      PORTC.OUTCLR = 0b110;
      break;
    case 4:
      PORTB.OUTCLR = (1<<0);
      PORTC.OUTCLR = (1<<0);
      break;
    case 5:
      PORTB.OUTCLR = 0b110;
      break;
    case 6:
      PORTB.OUTCLR = 0b11000;
      break;
      
  }
}

inline void set_motor_pin(uint8_t motor, bool dir, bool state){
  switch(motor){
    case 1:
      #ifndef SERIALDEBUG
      if(dir){
        if (state){
          PORTA.OUTSET = (1<<2);
        } else {
          PORTA.OUTCLR = (1<<2);
        }
      } else {
        if (state){
          PORTA.OUTSET = (1<<1);
        } else {
          PORTA.OUTCLR = (1<<1);
        }
      }
      #endif
      break;
    case 2:
      if(dir){
        if (state){
          PORTA.OUTSET = (1<<3);
        } else {
          PORTA.OUTCLR = (1<<3);
        }        
      } else {
        if (state){
          PORTC.OUTSET = (1<<3);
        } else {
          PORTC.OUTCLR = (1<<3);
        }
      }
      break;
    case 3:
      if(dir){
        if (state){
          PORTC.OUTSET = (1<<2);
        } else {
          PORTC.OUTCLR = (1<<2);
        }        
      } else {
        if (state){
          PORTC.OUTSET = (1<<1);
        } else {
          PORTC.OUTCLR = (1<<1);
        }
      }
      break;
    case 4:
      if(dir){
        if (state){
          PORTC.OUTSET = (1<<0);
        } else {
          PORTC.OUTCLR = (1<<0);
        }        
      } else {
        if (state){
          PORTB.OUTSET = (1<<0);
        } else {
          PORTB.OUTCLR = (1<<0);
        }
      }
      break;
    case 5:
      if(dir){
        if (state){
          PORTB.OUTSET = (1<<2);
        } else {
          PORTB.OUTCLR = (1<<2);
        }        
      } else {
        if (state){
          PORTB.OUTSET = (1<<1);
        } else {
          PORTB.OUTCLR = (1<<1);
        }
      }
      break;
    case 6:
      if(dir){
        if (state){
          PORTB.OUTSET = (1<<4);
        } else {
          PORTB.OUTCLR = (1<<4);
        }        
      } else {
        if (state){
          PORTB.OUTSET = (1<<3);
        } else {
          PORTB.OUTCLR = (1<<3);
        }
      }
      break;
  }
}

void toggle_motor_pin(uint8_t motor, bool dir){
  switch(motor){
    case 1:
      #ifndef SERIALDEBUG
      if(dir){
        PORTA.OUTTGL = (1<<2);
        PORTA.OUTCLR = (1<<1);
      } else {
        PORTA.OUTTGL = (1<<1);
        PORTA.OUTCLR = (1<<2);
      }
      #endif
      break;
    case 2:
      if(dir){
        PORTA.OUTTGL = (1<<3);
        PORTC.OUTCLR = (1<<3);
      } else {
        PORTC.OUTTGL = (1<<3);
        PORTA.OUTCLR = (1<<3);
      }
      break;
    case 3:
      if(dir){
        PORTC.OUTTGL = (1<<2);
        PORTC.OUTCLR = (1<<1);
      } else {
        PORTC.OUTTGL = (1<<1);
        PORTC.OUTCLR = (1<<2);
      }
      break;
    case 4:
      if(dir){
        PORTC.OUTTGL = (1<<0);
        PORTB.OUTCLR = (1<<0);
      } else {
        PORTB.OUTTGL = (1<<0);
        PORTC.OUTCLR = (1<<0);
      }
      break;
    case 5:
      if(dir){
        PORTB.OUTTGL = (1<<2);
        PORTB.OUTCLR = (1<<1);
      } else {
        PORTB.OUTTGL = (1<<1);
        PORTB.OUTCLR = (1<<2);
      }
      break;
    case 6:
      if(dir){
        PORTB.OUTTGL = (1<<4);
        PORTB.OUTCLR = (1<<3);
      } else {
        PORTB.OUTTGL = (1<<3);
        PORTB.OUTCLR = (1<<4);
      }
      break;
  }
}

ISR(PORTA_PORT_vect) {
  PORTA.INTFLAGS=255;
  bool state = PORTA.IN & (1 << 7);
  if (state){
    rcin[0].risetime = TCB0.CNT;    
  } else {
    // rcin[1].falltime = TCB0.CNT;
    rcin[0].pulse = TCB0.CNT-rcin[0].risetime;
    rcin[0].updated = true;
  }
}

ISR(PORTB_PORT_vect) {
  PORTB.INTFLAGS=255;
  bool state = PORTB.IN & (1 << 5);
  if (state){
    rcin[1].risetime = TCB0.CNT; 
  } else {
    rcin[1].pulse = TCB0.CNT-rcin[1].risetime;
    rcin[1].updated = true;
  }  
}

// compare interrupt for motor PWM generation
ISR(TCA0_CMP0_vect){
  // clear the interrupt flag of cmp0
  TCA0.SINGLE.INTFLAGS = (1 << 4);
  // turn the motor pin off
  set_motor_pin(state.motor,state.dir,0);
}

// timer overlfow interrupt for motor PWM generation
ISR(TCA0_OVF_vect){
  // clear the interrupt flag of OVF
  TCA0.SINGLE.INTFLAGS = (1 << 0);
  // turn the motor pin on
  set_motor_pin(state.motor,state.dir,1);
}