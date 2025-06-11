// modes SSC, PWM, SPC, HSM, AB_STEP, AB_PHASE 



// AB_PHASE implementaion 

int angle = 0;
int prev_b = 0;
void handle_test_angle() {
	int ifa = digitalRead(12);
	int ifb = digitalRead(11);
	int ifc = digitalRead(10);
	
	if(ifc == 1){
		angle = 0;
	}	

	if (prev_b == 1) {
		if (ifb == 0){
			prev_b = 0; 
		}
	} else {
		if (ifb == 1){
			prev_b = 1; 
			angle += -1 + ifa * 2;
		}

	}

	Serial.print("current angle is ");
	Serial.println(angle / 4096 * 360);

}