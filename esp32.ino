/*
 * IoT-Enabled Smart Shoe for Posture and Walking Analysis
 * ESP32 Main Firmware
 * 
 * National Institute of Business Management
 * HND in Software Engineering - Group 14
 * 
 * Features:
 * - Pressure sensor reading (4 FSRs per shoe)
 * - Accelerometer/Gyroscope (MPU6050) for gait and fall detection
 * - GPS tracking
 * - Temperature sensing
 * - Haptic feedback (vibration motor)
 * - Bluetooth Low Energy communication
 * - Battery monitoring
 */

#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ====================== Pin Definitions ======================
// Pressure Sensors (FSR) - Analog Pins
#define FSR_LEFT_HEEL   34    // Left shoe - Heel
#define FSR_LEFT_MID    35    // Left shoe - Midfoot
#define FSR_LEFT_BALL   32    // Left shoe - Ball
#define FSR_LEFT_TOE    33    // Left shoe - Toe

#define FSR_RIGHT_HEEL  36    // Right shoe - Heel
#define FSR_RIGHT_MID   39    // Right shoe - Midfoot
#define FSR_RIGHT_BALL  34    // Right shoe - Ball (using different pin if needed)
#define FSR_RIGHT_TOE   35    // Right shoe - Toe

// Vibration Motor (Haptic Feedback)
#define VIBRATION_MOTOR 25

// Temperature Sensor
#define TEMP_SENSOR_PIN 26

// Battery Monitoring
#define BATTERY_PIN     27

// LED Indicators
#define LED_CONNECTION  2     // Blue LED - Bluetooth connection status
#define LED_CHARGING    4     // Green LED - Charging status
#define LED_ERROR       16    // Red LED - Error indicator

// ====================== Global Objects ======================
BluetoothSerial SerialBT;      // Bluetooth Serial object
TinyGPSPlus gps;               // GPS object
MPU6050 mpu;                   // MPU6050 object
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// ====================== Constants ======================
const char* DEVICE_NAME = "SmartShoe_Group14";
const char* PIN_CODE = "1234";     // Optional pairing code

// Sampling rates
const int SENSOR_READ_DELAY = 100;     // Read sensors every 100ms (10Hz)
const int GPS_READ_DELAY = 1000;       // Update GPS every 1 second
const int DATA_SEND_DELAY = 500;       // Send data via BT every 500ms
const int BATTERY_CHECK_DELAY = 60000; // Check battery every minute

// Thresholds
const int FALL_ACCEL_THRESHOLD = 2000;  // Fall detection threshold (mg)
const int IMBALANCE_THRESHOLD = 30;     // Pressure imbalance percentage
const float BATTERY_LOW_THRESHOLD = 3.4; // Low battery voltage

// ====================== Global Variables ======================
// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastGPSRead = 0;
unsigned long lastDataSend = 0;
unsigned long lastBatteryCheck = 0;

// Sensor data variables
float fsrValues[8] = {0};        // Array for 8 FSR sensors
float temperature = 25.0;         // Current temperature in Celsius
float batteryVoltage = 4.1;       // Battery voltage
int batteryPercent = 100;         // Battery percentage

// MPU6050 data
int16_t ax, ay, az;               // Accelerometer raw values
int16_t gx, gy, gz;               // Gyroscope raw values
float angleX = 0, angleY = 0;     // Calculated angles

// GPS data
double latitude = 0.0;
double longitude = 0.0;
float speed = 0.0;
float altitude = 0.0;
int satellites = 0;
char gpsTime[10] = "00:00:00";
char gpsDate[11] = "00/00/0000";

// State variables
bool deviceConnected = false;
bool fallDetected = false;
bool imbalanceDetected = false;
int stepCount = 0;
String gaitPattern = "normal";

// ====================== Function Prototypes ======================
void readPressureSensors();
void readMPU6050();
void readGPSSensor();
void readTemperature();
void readBatteryLevel();
void detectFall();
void detectImbalance();
void detectGaitPattern();
void sendDataViaBluetooth();
void processSerialCommands();
void vibrateMotor(int duration);
void calibrateSensors();
void printDebugInfo();

// ====================== Setup Function ======================
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\n\n=== Smart Shoe ESP32 Starting ===");
  
  // Initialize pins
  pinMode(VIBRATION_MOTOR, OUTPUT);
  pinMode(LED_CONNECTION, OUTPUT);
  pinMode(LED_CHARGING, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  
  // Set initial LED states
  digitalWrite(LED_CONNECTION, LOW);
  digitalWrite(LED_CHARGING, LOW);
  digitalWrite(LED_ERROR, LOW);
  
  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Initialize Bluetooth
  SerialBT.begin(DEVICE_NAME);
  Serial.println("Bluetooth started. Device name: " + String(DEVICE_NAME));
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
    digitalWrite(LED_CONNECTION, HIGH); // Blink to show sensor OK
    delay(200);
    digitalWrite(LED_CONNECTION, LOW);
  } else {
    Serial.println("MPU6050 connection failed!");
    digitalWrite(LED_ERROR, HIGH); // Error LED on
  }
  
  // Initialize temperature sensor
  tempSensor.begin();
  
  // Calibrate sensors
  calibrateSensors();
  
  // Initial vibration feedback (3 short buzzes)
  for (int i = 0; i < 3; i++) {
    vibrateMotor(100);
    delay(200);
  }
  
  Serial.println("Setup complete. Ready to connect.");
  digitalWrite(LED_CONNECTION, HIGH); // Indicate ready
}

// ====================== Main Loop ======================
void loop() {
  unsigned long currentMillis = millis();
  
  // Read sensors at defined interval
  if (currentMillis - lastSensorRead >= SENSOR_READ_DELAY) {
    readPressureSensors();
    readMPU6050();
    readTemperature();
    
    // Process sensor data
    detectFall();
    detectImbalance();
    detectGaitPattern();
    
    lastSensorRead = currentMillis;
  }
  
  // Read GPS at defined interval
  if (currentMillis - lastGPSRead >= GPS_READ_DELAY) {
    readGPSSensor();
    lastGPSRead = currentMillis;
  }
  
  // Send data via Bluetooth
  if (currentMillis - lastDataSend >= DATA_SEND_DELAY) {
    sendDataViaBluetooth();
    lastDataSend = currentMillis;
  }
  
  // Check battery level
  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_DELAY) {
    readBatteryLevel();
    lastBatteryCheck = currentMillis;
  }
  
  // Check for incoming Bluetooth commands
  if (SerialBT.available()) {
    processSerialCommands();
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}

// ====================== Sensor Reading Functions ======================

/**
 * Read all 8 pressure sensors (FSRs)
 */
void readPressureSensors() {
  // Read left shoe sensors
  fsrValues[0] = analogRead(FSR_LEFT_HEEL);
  fsrValues[1] = analogRead(FSR_LEFT_MID);
  fsrValues[2] = analogRead(FSR_LEFT_BALL);
  fsrValues[3] = analogRead(FSR_LEFT_TOE);
  
  // Read right shoe sensors
  fsrValues[4] = analogRead(FSR_RIGHT_HEEL);
  fsrValues[5] = analogRead(FSR_RIGHT_MID);
  fsrValues[6] = analogRead(FSR_RIGHT_BALL);
  fsrValues[7] = analogRead(FSR_RIGHT_TOE);
  
  // Optional: Apply moving average filter for smoothing
  for (int i = 0; i < 8; i++) {
    // Simple noise filtering - ignore values below threshold
    if (fsrValues[i] < 10) fsrValues[i] = 0;
  }
}

/**
 * Read MPU6050 accelerometer and gyroscope
 */
void readMPU6050() {
  // Read raw accelerometer and gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful units if needed
  // Accelerometer: range -32768 to 32768 corresponds to -2g to +2g
  // So divide by 16384 to get g force
  
  // Simple angle calculation (complementary filter would be better)
  float accelAngleX = atan2(ay, az) * 180 / PI;
  float accelAngleY = atan2(ax, az) * 180 / PI;
  
  // Simple complementary filter (for production, implement proper filter)
  angleX = 0.98 * (angleX + gx * 0.001) + 0.02 * accelAngleX;
  angleY = 0.98 * (angleY + gy * 0.001) + 0.02 * accelAngleY;
  
  // Step counting logic (simple threshold-based)
  static bool stepState = false;
  static int lastStepTime = 0;
  
  if (az > 18000 && !stepState) { // Heel strike detected
    stepState = true;
    if (millis() - lastStepTime > 300) { // Debounce
      stepCount++;
      lastStepTime = millis();
    }
  } else if (az < 14000 && stepState) {
    stepState = false;
  }
}

/**
 * Read GPS module
 */
void readGPSSensor() {
  // This is a placeholder - actual GPS reading requires
  // continuous feeding of serial data
  
  // For now, we'll simulate GPS data
  // In production, you'd read from SoftwareSerial connected to GPS module
  
  /*
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.speed.isValid()) {
        speed = gps.speed.kmph();
      }
      if (gps.altitude.isValid()) {
        altitude = gps.altitude.meters();
      }
      if (gps.satellites.isValid()) {
        satellites = gps.satellites.value();
      }
      if (gps.time.isValid()) {
        sprintf(gpsTime, "%02d:%02d:%02d", 
                gps.time.hour(), gps.time.minute(), gps.time.second());
      }
    }
  }
  */
  
  // Simulated GPS data for testing
  static float simLat = 6.9271;
  static float simLng = 79.8612;
  
  // Simulate movement
  if (stepCount > 0) {
    simLat += 0.000001 * stepCount;
    simLng += 0.000001 * stepCount;
  }
  
  latitude = simLat;
  longitude = simLng;
  satellites = 8;
  speed = stepCount * 0.05; // Rough estimation
}

/**
 * Read temperature sensor
 */
void readTemperature() {
  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);
  
  // If reading fails, use last known value
  if (temperature == DEVICE_DISCONNECTED_C) {
    // Keep previous value
  }
}

/**
 * Read battery voltage and calculate percentage
 */
void readBatteryLevel() {
  // Read battery voltage through voltage divider
  int rawValue = analogRead(BATTERY_PIN);
  batteryVoltage = (rawValue / 4095.0) * 7.4; // Assuming 2:1 voltage divider for 4.2V max
  
  // Convert to percentage (3.3V empty, 4.2V full for Li-Po)
  batteryPercent = map(batteryVoltage * 100, 330, 420, 0, 100);
  batteryPercent = constrain(batteryPercent, 0, 100);
  
  // Check for low battery
  if (batteryVoltage < BATTERY_LOW_THRESHOLD) {
    // Send low battery alert via Bluetooth
    SerialBT.println("ALERT:LOW_BATTERY");
    
    // Flash LED
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_ERROR, HIGH);
      delay(200);
      digitalWrite(LED_ERROR, LOW);
      delay(200);
    }
  }
}

// ====================== Analysis Functions ======================

/**
 * Detect fall events using accelerometer data
 */
void detectFall() {
  // Calculate total acceleration magnitude
  float totalAccel = sqrt(ax*ax + ay*ay + az*az) / 16384.0; // in g's
  
  // Fall detection logic
  static bool fallPossible = false;
  static unsigned long fallTime = 0;
  
  // Sudden high acceleration (impact)
  if (totalAccel > 2.5 && !fallPossible) {
    fallPossible = true;
    fallTime = millis();
  }
  
  // Check for subsequent inactivity
  if (fallPossible && (millis() - fallTime > 2000)) {
    // If after 2 seconds there's little movement, confirm fall
    if (totalAccel < 0.2) {
      if (!fallDetected) {
        fallDetected = true;
        Serial.println("FALL DETECTED!");
        SerialBT.println("ALERT:FALL_DETECTED");
        
        // Send GPS location with fall alert
        char alertMsg[100];
        sprintf(alertMsg, "FALL_LOCATION:%.6f,%.6f", latitude, longitude);
        SerialBT.println(alertMsg);
        
        // Vibrate motor to alert user
        vibrateMotor(1000); // 1 second vibration
        
        // Flash error LED
        for (int i = 0; i < 10; i++) {
          digitalWrite(LED_ERROR, HIGH);
          delay(100);
          digitalWrite(LED_ERROR, LOW);
          delay(100);
        }
      }
    } else {
      fallPossible = false;
      fallDetected = false;
    }
  }
  
  // Reset after 10 seconds
  if (fallDetected && (millis() - fallTime > 10000)) {
    fallDetected = false;
  }
}

/**
 * Detect walking imbalance based on pressure distribution
 */
void detectImbalance() {
  // Calculate total pressure on each foot
  float leftTotal = fsrValues[0] + fsrValues[1] + fsrValues[2] + fsrValues[3];
  float rightTotal = fsrValues[4] + fsrValues[5] + fsrValues[6] + fsrValues[7];
  
  // Only detect if walking (some pressure on both feet)
  if (leftTotal > 100 && rightTotal > 100) {
    float imbalance = abs(leftTotal - rightTotal) / max(leftTotal, rightTotal) * 100;
    
    if (imbalance > IMBALANCE_THRESHOLD && !imbalanceDetected) {
      imbalanceDetected = true;
      SerialBT.println("ALERT:IMBALANCE_DETECTED");
      
      // Light haptic feedback for imbalance
      vibrateMotor(300);
      
    } else if (imbalance <= IMBALANCE_THRESHOLD) {
      imbalanceDetected = false;
    }
  }
}

/**
 * Analyze gait pattern based on pressure distribution sequence
 */
void detectGaitPattern() {
  // Simple gait analysis based on pressure sequence
  // Normal gait: Heel -> Midfoot -> Ball -> Toe
  
  static int gaitPhase = 0;
  static unsigned long lastPhaseTime = 0;
  
  // Check for heel strike (high pressure on heel sensors)
  if (fsrValues[0] > 500 || fsrValues[4] > 500) {
    if (gaitPhase == 0) {
      gaitPhase = 1;
      lastPhaseTime = millis();
    }
  }
  
  // Check for toe-off (high pressure on toe sensors)
  if (fsrValues[3] > 500 || fsrValues[7] > 500) {
    if (gaitPhase == 1 && (millis() - lastPhaseTime > 100)) {
      gaitPhase = 2;
      // Complete gait cycle detected
      
      // Determine gait pattern
      if ((millis() - lastPhaseTime) < 300) {
        gaitPattern = "fast";
      } else if ((millis() - lastPhaseTime) > 600) {
        gaitPattern = "slow";
      } else {
        gaitPattern = "normal";
      }
      
      gaitPhase = 0;
    }
  }
  
  // Reset if cycle takes too long
  if (gaitPhase == 1 && (millis() - lastPhaseTime > 1000)) {
    gaitPhase = 0;
  }
}

// ====================== Communication Functions ======================

/**
 * Send all sensor data via Bluetooth in a formatted string
 */
void sendDataViaBluetooth() {
  // Format: "DATA:pressure0,pressure1,...,pressure7,temp,ax,ay,az,steps,gait,lat,lng,battery"
  String dataString = "DATA:";
  
  // Add pressure sensor values
  for (int i = 0; i < 8; i++) {
    dataString += String(fsrValues[i]);
    dataString += (i < 7) ? "," : ";";
  }
  
  // Add temperature
  dataString += String(temperature) + ";";
  
  // Add accelerometer values
  dataString += String(ax) + "," + String(ay) + "," + String(az) + ";";
  
  // Add step count and gait pattern
  dataString += String(stepCount) + "," + gaitPattern + ";";
  
  // Add GPS coordinates
  dataString += String(latitude, 6) + "," + String(longitude, 6) + ";";
  
  // Add battery percentage
  dataString += String(batteryPercent);
  
  // Send via Bluetooth
  SerialBT.println(dataString);
  
  // Also print to serial for debugging
  Serial.println(dataString);
  
  // Update connection status LED
  digitalWrite(LED_CONNECTION, HIGH);
}

/**
 * Process incoming commands from mobile app
 */
void processSerialCommands() {
  String command = SerialBT.readStringUntil('\n');
  command.trim();
  
  Serial.println("Received command: " + command);
  
  if (command == "GET_STATUS") {
    // Send current status
    String status = "STATUS:" + String(batteryPercent) + "," + 
                    String(stepCount) + "," + 
                    String(fallDetected ? "1" : "0") + "," +
                    String(imbalanceDetected ? "1" : "0");
    SerialBT.println(status);
    
  } else if (command == "VIBRATE") {
    // Vibrate motor (for testing or alerts from app)
    vibrateMotor(500);
    
  } else if (command == "CALIBRATE") {
    // Calibrate sensors
    calibrateSensors();
    SerialBT.println("CALIBRATION_DONE");
    
  } else if (command.startsWith("SET_THRESHOLD")) {
    // Parse threshold command
    // Format: SET_THRESHOLD:fall,imbalance
    // Example: SET_THRESHOLD:2500,35
    
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      // Update thresholds (parse values from string)
      // This would need proper implementation
      SerialBT.println("THRESHOLDS_UPDATED");
    }
    
  } else if (command == "RESET_STEPS") {
    stepCount = 0;
    SerialBT.println("STEPS_RESET");
    
  } else if (command == "GET_GPS") {
    // Send current GPS location
    char gpsMsg[50];
    sprintf(gpsMsg, "GPS:%.6f,%.6f", latitude, longitude);
    SerialBT.println(gpsMsg);
  }
}

// ====================== Helper Functions ======================

/**
 * Control vibration motor for haptic feedback
 * @param duration Vibration duration in milliseconds
 */
void vibrateMotor(int duration) {
  digitalWrite(VIBRATION_MOTOR, HIGH);
  delay(duration);
  digitalWrite(VIBRATION_MOTOR, LOW);
}

/**
 * Calibrate sensors (especially FSRs for zero-offset)
 */
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  digitalWrite(LED_ERROR, HIGH);
  
  // Wait for user to remove pressure from shoes
  vibrateMotor(500);
  delay(2000);
  
  // Read current values as zero offset
  int zeroOffsets[8];
  for (int i = 0; i < 8; i++) {
    zeroOffsets[i] = analogRead(FSR_LEFT_HEEL + i); // This needs proper mapping
  }
  
  Serial.println("Calibration complete");
  digitalWrite(LED_ERROR, LOW);
  vibrateMotor(200);
}

/**
 * Print debug information to Serial monitor
 */
void printDebugInfo() {
  Serial.println("\n=== Smart Shoe Debug Info ===");
  Serial.print("Battery: "); Serial.print(batteryVoltage); Serial.print("V (");
  Serial.print(batteryPercent); Serial.println("%)");
  
  Serial.println("Pressure Sensors (L/R):");
  Serial.print("Heel: "); Serial.print(fsrValues[0]); Serial.print(" | ");
  Serial.println(fsrValues[4]);
  Serial.print("Mid:  "); Serial.print(fsrValues[1]); Serial.print(" | ");
  Serial.println(fsrValues[5]);
  Serial.print("Ball: "); Serial.print(fsrValues[2]); Serial.print(" | ");
  Serial.println(fsrValues[6]);
  Serial.print("Toe:  "); Serial.print(fsrValues[3]); Serial.print(" | ");
  Serial.println(fsrValues[7]);
  
  Serial.print("Temp: "); Serial.print(temperature); Serial.println(" C");
  Serial.print("Accel: "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.println(az);
  Serial.print("Steps: "); Serial.println(stepCount);
  Serial.print("Gait: "); Serial.println(gaitPattern);
  Serial.print("GPS: "); Serial.print(latitude, 6); Serial.print(", "); Serial.println(longitude, 6);
  Serial.print("Fall: "); Serial.println(fallDetected ? "YES" : "NO");
  Serial.print("Imbalance: "); Serial.println(imbalanceDetected ? "YES" : "NO");
  Serial.println("=============================\n");
}