void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);  // Replace with your actual pins if using SoftwareSerial

  delay(1000);
  Serial.println("Sending message to Pico W...");
  Serial2.println("Hello from Arduino!");
}

void loop() {
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    Serial.print("Echo from Pico W: ");
    Serial.println(response);
    delay(1000);  // Send periodically
    Serial2.println("Ping!");
  }
}
