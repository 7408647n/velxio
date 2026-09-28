// pad-probe: what a released pad reads with a module's pull resistor on it.
// The circuit the tab maps for this sketch: a module pull-up on GPIO 18 and a
// module pull-down on GPIO 19, nothing else on either line.
// Fixture of test/backend/integration/test_esp32_qemu_pads.py.
void setup() {
  Serial.begin(115200);
}

void loop() {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  int outLow = digitalRead(18);
  pinMode(18, INPUT);                 // released: the module's pull-up
  delayMicroseconds(5);
  int releasedUp = digitalRead(18);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  int outHigh = digitalRead(19);
  pinMode(19, INPUT);                 // released: the module's pull-down
  delayMicroseconds(5);
  int releasedDown = digitalRead(19);

  pinMode(18, OUTPUT);                // the guest drives LOW over the pull-up
  digitalWrite(18, LOW);
  delayMicroseconds(5);
  int stillLow = digitalRead(18);

  pinMode(18, INPUT_PULLDOWN);        // the module's 10k beats the pad's 45k
  delayMicroseconds(5);
  int moduleBeatsPad = digitalRead(18);

  Serial.printf("PADS out_low=%d released_up=%d out_high=%d released_down=%d still_low=%d module_beats_pad=%d\n",
                outLow, releasedUp, outHigh, releasedDown, stillLow, moduleBeatsPad);
  delay(300);
}
