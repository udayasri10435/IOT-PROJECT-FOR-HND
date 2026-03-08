#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Pin Definitions
const int FSR_HEEL = 34;      // FSR connected to Analog pin (ESP32) [cite: 33]
const int FSR_TOE = 35;       // FSR connected to Analog pin [cite: 33]
const int VIBRO_MOTOR = 26;   // Vibration motor for haptic feedback [cite: 37]
const int RX_PIN = 16, TX_PIN = 17; // GPS Pins [cite: 35]

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial ss(RX_PIN, TX_PIN);

void setup() {
  Serial.begin(115200);
  ss.begin(9600); // GPS Baud Rate [cite: 35]
  
  // Initialize Accelerometer [cite: 34]
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) yield();
  }

  pinMode(VIBRO_MOTOR, OUTPUT);
  Serial.println("Smart Shoe System Initialized...");
}

void loop() {
  // 1. Read Pressure Sensors [cite: 48, 51]
  int heelPressure = analogRead(FSR_HEEL);
  int toePressure = analogRead(FSR_TOE);

  // 2. Read Accelerometer for Gait & Fall Detection [cite: 34, 57]
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Simple Fall Detection Logic [cite: 57, 104]
  float totalAccel = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
  if (totalAccel > 25.0) { // Threshold for sudden impact [cite: 57]
    digitalWrite(VIBRO_MOTOR, HIGH); // Haptic alert [cite: 59]
    Serial.println("ALERT: Fall Detected!"); [cite: 55]
    delay(1000);
    digitalWrite(VIBRO_MOTOR, LOW);
  }

  // 3. Read GPS Location [cite: 35, 58]
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        Serial.print("Lat: "); Serial.println(gps.location.lat(), 6); [cite: 58]
      }
    }
  }

  // 4. Output Data for Mobile App (Firebase/Bluetooth) [cite: 52, 62]
  // In a real scenario, you'd send this via WiFi to Firebase [cite: 62]
  Serial.print("Data: ");
  Serial.print(heelPressure); Serial.print(",");
  Serial.print(toePressure); Serial.print(",");
  Serial.println(totalAccel);

  delay(500); // Processing interval [cite: 51]
}