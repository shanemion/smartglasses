// Seeed XIAO ESP32S3 Sense - D0 Digital Output
// Sends HIGH signal every 2 seconds

#define D0_PIN 1  // D0 corresponds to GPIO 0 on XIAO ESP32S3

void setup() {
  // Initialize serial communication for debugging (optional)
  Serial.begin(115200);
  
  // Set D0 as digital output
  pinMode(D0_PIN, OUTPUT);
  
  // Start with pin LOW
  digitalWrite(D0_PIN, LOW);
  
  Serial.println("XIAO ESP32S3 - D0 Output initialized");
}

void loop() {
  // Send HIGH signal
  digitalWrite(D0_PIN, HIGH);
  Serial.println("D0 -> HIGH");
  
  // Keep HIGH for 100ms (brief pulse)
  delay(100);
  
  // Set back to LOW
  digitalWrite(D0_PIN, LOW);
  Serial.println("D0 -> LOW");
  
  // Wait for 2 seconds before next HIGH signal
  delay(1900);  // 1900ms + 100ms HIGH = 2000ms total
}
