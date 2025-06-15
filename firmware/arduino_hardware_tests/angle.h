// modes SSC, PWM, SPC, HSM, AB_STEP, AB_PHASE 


// AB_PHASE implementaion 
#define ANG_A 14   // white wire
#define ANG_B 25   // yellow wire
#define ANG_C 26   // green wire, should be connected with 1kOm lol), idk

#define STEP 0.08789

bool prev_b = 0;
double angle = 0;

void IRAM_ATTR isr() {
  angle = 0;
  delay(0);
}

long prev_tick_ang = 0;
void handle_test_angle() {
	bool ifa = digitalRead(ANG_A);
	bool ifb = digitalRead(ANG_B);
	bool ifc = digitalRead(ANG_C);

	if (prev_b == 1) {
		if (ifb == 0){
			prev_b = 0; 
		}
	} else {
		if (ifb == 1){
			prev_b = 1; 
			angle -= STEP * (1 - ifa * 2);
		}
	}

  if (millis() - prev_tick_ang >= 1000){
    prev_tick_ang = millis();
    Serial.print("Ang: ");
    Serial.println(angle);
  }
}