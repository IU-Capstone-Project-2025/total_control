// Modes of usage TEST-ANGLE, TEST-ENCODER, TEST-MOTOR

#define TEST-ANGLE      
//#define TEST-ENCODER
//#define TEST-MOTOR


#ifdef TEST-ANGLE
#include "angle.h"
#endif


// TODO ecoder.h and motor.h
#ifdef TEST-ENCODER
#include "encoder.h"
#endif

#ifdef TEST-MOTOR
#include "motor.h"
#endif


void setup() {
  Serial.begin(9600);

  #ifdef TEST-ANGLE
  // pinmodes if needed
  #endif

  #ifdef TEST-ENCODER
  // pinmodes if needed
  #endif

  #ifdef TEST-MOTOR
  // pinmodes if needed
  #endif
}

void loop() {
  #ifdef TEST-ANGLE
  handle_test_angle();
  #endif

  #ifdef TEST-ENCODER
  handle_test_encoder();
  #endif
  
  #ifdef TEST-MOTOR
  handle_test_motor();
  #endif
  
  
}
