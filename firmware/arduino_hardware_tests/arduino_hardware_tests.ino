#include "esp32-hal-cpu.h"

/*       
            Modes of usage TEST-ANGLE, TEST-ENCODER, TEST-MOTOR     
*/

#define TEST_ANGLE      
#define TEST_ENCODER
//#define TEST_MOTOR



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
  Serial.begin(921600);
  setCpuFrequencyMhz(240);


  #ifdef TEST_ANGLE
  pinMode(ANG_A, INPUT_PULLDOWN);
  pinMode(ANG_B, INPUT_PULLDOWN);
  pinMode(ANG_C, INPUT_PULLDOWN);  
  attachInterrupt(ANG_C, zero_angle, RISING);
  attachInterrupt(ANG_B, change_angle, RISING);
  #endif

  #ifdef TEST_ENCODER
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
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