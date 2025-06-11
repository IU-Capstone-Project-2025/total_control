const int pinA = 6;  // Канал A (прерывание 0 на Uno/Nano)
const int pinB = 7;  // Канал B (прерывание 1)
volatile int counter = 0;
int lastStateA;

void setup() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  Serial.begin(9600);  // Более стандартная скорость
  attachInterrupt(digitalPinToInterrupt(pinA), readEncoder, CHANGE);
  lastStateA = digitalRead(pinA);
}

void readEncoder() {
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);
  
  if (stateA != lastStateA) {
    counter += (stateB != stateA) ? 1 : -1;
    Serial.print("Pos: ");
    Serial.println(counter);
    // Или просто:
    // Serial.println(counter);
  }
  lastStateA = stateA;
}

void loop() {
  // Пустой цикл — всё работает в прерывании
}