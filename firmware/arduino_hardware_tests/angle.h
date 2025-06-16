// modes SSC, PWM, SPC, HSM, AB_STEP, AB_PHASE 


// AB_PHASE implementaion 
#define ANG_A 14   // white wire
#define ANG_B 25   // yellow wire
#define ANG_C 26   // green wire, should be connected with 1kOm

#define STEP 0.08789

double angle = 0;

void IRAM_ATTR zero_angle() {
  angle = 0;
  delay(0);
}

void IRAM_ATTR change_angle() {
  bool ifa = digitalRead(ANG_A);
  angle -= STEP * (1 - ifa * 2);
  delay(0);
}

long prev_tick_ang = 0;
double prev_angle = 0;
void handle_test_angle() {
  if (millis() - prev_tick_ang >= 5){
    prev_tick_ang = millis();

    double vel = (angle - prev_angle) * 100;
    prev_angle = angle;
    
    Serial.print("Ang: ");
    Serial.print(angle);
    Serial.print(", Vel: ");
    Serial.println(vel);
  }
}