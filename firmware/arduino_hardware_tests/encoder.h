#define ENC_A 25
#define ENC_B 26


volatile int counter = 0;

// функция в интерапте (не должно быть Serial а то насрет)
void readEncoder() {
  int stateA = digitalRead(ENC_A);
  int stateB = digitalRead(ENC_B);
  counter += (stateB != stateA) ? 1 : -1;
  delay(0);
}


long prev_tick_en = 0;
void handle_test_encoder() {
  // вывод значений в серийный порт каждую секунду
  if (millis() - prev_tick_en >= 1000){
    prev_tick_en = millis();
    Serial.print("Pos: ");
    Serial.println(counter);
  }
}


