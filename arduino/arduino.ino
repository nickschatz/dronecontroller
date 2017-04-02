#include <SoftwareSerial.h>

SoftwareSerial esp(2, 3);

void setup() {
  Serial.begin(230400);
  while (!Serial) {
    ;
  }
  
  esp.begin(115200);
}

void loop() {
  if (esp.available()) {
    Serial.write(esp.read());
  }

}
