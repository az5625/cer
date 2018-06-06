/*
  CER - Complex Envelope Recorder

  This device is used to record and playback 0-5V envelopes.
  Creative Commons License
  CER by Pantala Labs is licensed
  under a Creative Commons Attribution 4.0 International License.
  Gibran Curtiss Salomao. MAR/2018 - CC-BY-SA
*/
void triTableGen() {
  unsigned int value = 0;
  unsigned int stepValue = 16;

  Serial.println("const PROGMEM int triwave[512] =");
  Serial.println("{");
  for (int i = 1; i <= 256; i++) {
    Serial.print(value);
    Serial.print(",");
    value = value + stepValue;
    if ((i % 16) == 0) {
      Serial.println(" ");
    }
  }
  stepValue = -stepValue;
  for (int i = 1; i <= 256; i++) {
    Serial.print(value);
    Serial.print(",");
    value = value + stepValue;
    if ((i % 16) == 0) {
      Serial.println(" ");
    }
  }
  Serial.println("};");
}

void rampTableGen() {
  unsigned int value = 0;
  unsigned int stepValue = 8;

  Serial.println("const PROGMEM int rampwave[512] =");
  Serial.println("{");
  for (int i = 1; i <= 512; i++) {
    Serial.print(value);
    Serial.print(",");
    value = value + stepValue;
    if ((i % 16) == 0) {
      Serial.println(" ");
    }
  }
  Serial.println("};");
}

void sawTableGen() {
  unsigned int value = 4088;
  unsigned int stepValue = -8;

  Serial.println("const PROGMEM int sawwave[512] =");
  Serial.println("{");
  for (int i = 1; i <= 512; i++) {
    Serial.print(value);
    Serial.print(",");
    value = value + stepValue;
    if ((i % 16) == 0) {
      Serial.println(" ");
    }
  }
  Serial.println("};");
}

void sqrTableGen() {
  unsigned int value = 4088;
  unsigned int stepValue = 16;

  Serial.println("const PROGMEM int sqrwave[512] =");
  Serial.println("{");

  for (int i = 1; i <= 256; i++) {
    Serial.print(value);
    Serial.print(",");
    if ((i % 8) == 0) {
      Serial.println(" ");
    }
  }
  value = 0;
  for (int i = 1; i <= 256; i++) {
    Serial.print(value);
    Serial.print(",");
    if ((i % 8) == 0) {
      Serial.println(" ");
    }
  }
  Serial.println("};");
}

