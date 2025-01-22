// SIMPLE BLINK FOR ESP32 S3
const int LED = 2;
const int waitTime = 200;

void setup() {
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);
  delay(waitTime);                    
  digitalWrite(LED, LOW); 
  delay(waitTime);
}
