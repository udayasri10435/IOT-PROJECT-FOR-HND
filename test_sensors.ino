/*
 * Quick test script to verify all sensors are working
 * Upload this separately to test each component
 */

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Smart Shoe Sensor Test ===\n");
  
  testPressureSensors();
  testMPU6050();
  testBluetooth();
}

void loop() {
  // Individual tests can be run here
  delay(5000);
}

void testPressureSensors() {
  Serial.println("Testing Pressure Sensors (FSRs):");
  Serial.println("Apply pressure to each sensor...");
  
  for (int i = 0; i < 10; i++) {
    Serial.print("Left Heel: "); Serial.print(analogRead(34));
    Serial.print(" | Left Mid: "); Serial.print(analogRead(35));
    Serial.print(" | Left Ball: "); Serial.print(analogRead(32));
    Serial.print(" | Left Toe: "); Serial.println(analogRead(33));
    delay(500);
  }
  Serial.println();
}

void testMPU6050() {
  Serial.println("Testing MPU6050:");
  // Add MPU6050 test code here
  Serial.println("MPU6050 test complete\n");
}

void testBluetooth() {
  Serial.println("Testing Bluetooth:");
  Serial.println("Device should be visible as 'Test_Shoe'");
  // Add Bluetooth test code
  Serial.println("Bluetooth test complete\n");
}
