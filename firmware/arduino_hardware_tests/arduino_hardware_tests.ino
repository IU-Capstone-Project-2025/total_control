#include "esp32-hal-cpu.h"

/*       
            Modes of usage TEST-ANGLE, TEST-ENCODER, TEST-MOTOR     
                      CHOOSE ONE!!!!!!!
*/

#define TEST_ANGLE      
//#define TEST-ENCODER
//#define TEST-MOTOR



#ifdef TEST_ANGLE
#include "angle.h"
#endif

#ifdef TEST_ENCODER
#include "encoder.h"
#endif

#ifdef TEST_MOTOR
#include "motor.h"
#endif


// pin's defines are located in .h files 
void setup() {

  #ifdef TEST_ANGLE
  Serial.begin(9600);
  setCpuFrequencyMhz(240);
  pinMode(ANG_A, INPUT_PULLDOWN);
  pinMode(ANG_B, INPUT_PULLDOWN);
  pinMode(ANG_C, INPUT_PULLDOWN);  
  attachInterrupt(ANG_C, isr, RISING);
  #endif

  #ifdef TEST_ENCODER
  Serial.begin(9600);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(ENC_A, readEncoder, CHANGE);
  #endif

  #ifdef TEST_MOTOR
  // pinmodes and interrupts if needed
  #endif
}


// do not modify 
void loop() {
  #ifdef TEST_ANGLE
  handle_test_angle();
  #endif
  #ifdef TEST_ENCODER
  handle_test_encoder();
  #endif
  #ifdef TEST_MOTOR
  handle_test_motor();
  #endif
}