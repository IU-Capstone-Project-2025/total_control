#define ENC_A 16  // red wire
#define ENC_B 17  // yellow wire


int counter = 0;

void readEncoder() {
  int stateA = digitalRead(ENC_A);
  int stateB = digitalRead(ENC_B);
  counter += (stateB != stateA) ? 1 : -1;
  delay(0);
}


long prev_tick_en = 0;
int prev_counter = 0;
// int prev_velocity = 0;

void handle_test_encoder() {
  if (millis() - prev_tick_en >= 5){
    prev_tick_en = millis();

    int vel = (counter - prev_counter) * 100;
    prev_counter = counter;

    // int acc = (vel - prev_velocity) * 100;
    // prev_velocity = vel;
    
    Serial.print("Pos: ");
    Serial.print(counter);
    Serial.print(", Vel: ");
    Serial.println(vel);
  }
}


