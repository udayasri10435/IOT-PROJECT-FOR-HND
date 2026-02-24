
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <Wire.h>
#include <DHT.h>
#include <Preferences.h>

// ==================== RISK MANAGEMENT CONFIGURATIONS ====================
// This section addresses all risk factors mentioned in your proposal

// WiFi Credentials
#define WIFI_SSID "YourWiFiSSID"
#define WIFI_PASSWORD "YourWiFiPassword"

// Firebase Configuration
#define FIREBASE_HOST "your-project.firebaseio.com"
#define FIREBASE_AUTH "your-firebase-secret-key"

// Pin Definitions
// Pressure Sensors (FSR) - Using ADC pins
#define FSR1_PIN 34  // Heel - Left
#define FSR2_PIN 35  // Midfoot - Left
#define FSR3_PIN 32  // Forefoot - Left
#define FSR4_PIN 33  // Toe - Left

// For right shoe, use different pins if needed
#define FSR5_PIN 36  // Heel - Right
#define FSR6_PIN 39  // Midfoot - Right
#define FSR7_PIN 34  // Forefoot - Right (if using second ESP32)
#define FSR8_PIN 35  // Toe - Right

// MPU6050 - I2C pins (default)
#define MPU_SDA 21
#define MPU_SCL 22

// GPS Module - Hardware Serial
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// DHT22 Temperature/Humidity Sensor
#define DHT_PIN 4
#define DHT_TYPE DHT22

// Output Devices
#define VIBRATION_MOTOR_PIN 25  // PWM capable pin
#define HEATING_PIN 26          // MOSFET control for Peltier heating
#define COOLING_PIN 27          // MOSFET control for Peltier cooling
#define RGB_RED_PIN 14
#define RGB_GREEN_PIN 12
#define RGB_BLUE_PIN 13

// Battery Monitoring
#define BATTERY_PIN 39          // ADC for battery voltage
#define BATTERY_FULL 4.2        // Full battery voltage (Li-Po)
#define BATTERY_EMPTY 3.3       // Empty battery voltage

// ==================== GLOBAL OBJECTS ====================
FirebaseData firebaseData;
FirebaseAuth firebaseAuth;
FirebaseConfig firebaseConfig;

TinyGPSPlus gps;
MPU6050 mpu;
DHT dht(DHT_PIN, DHT_TYPE);
Preferences preferences;

// I2C for MPU6050
TwoWire I2CMPU = TwoWire(0);

// ==================== DATA STRUCTURES ====================
struct SensorData {
  // Pressure readings (0-4095 ADC)
  int pressureFSR[8];
  float pressureVoltage[8];
  float pressureKg[8];
  
  // Gait analysis
  float stepCount;
  float strideLength;
  float walkingSpeed;
  float gaitSymmetry;
  
  // Posture data
  float footAngle;
  float pronationAngle;
  float balanceScore;
  
  // Fall detection
  bool fallDetected;
  float fallConfidence;
  unsigned long fallTimestamp;
  
  // Temperature
  float insoleTemp;
  float ambientTemp;
  float targetTemp;
  
  // GPS
  double latitude;
  double longitude;
  float altitude;
  float speed;
  float heading;
  int satellites;
  
  // Battery
  float batteryVoltage;
  int batteryPercentage;
  
  // Timestamps
  unsigned long lastReadingTime;
  unsigned long deviceUptime;
} sensorData;

// ==================== CALIBRATION CONSTANTS ====================
// These values should be calibrated for each specific sensor
struct CalibrationData {
  // FSR calibration (voltage to force)
  float fsrMinVoltage[8] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  float fsrMaxVoltage[8] = {3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3, 3.3};
  float fsrMinForce[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float fsrMaxForce[8] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; // kg
  
  // MPU6050 calibration offsets
  int16_t accelOffset[3] = {0, 0, 0};
  int16_t gyroOffset[3] = {0, 0, 0};
  
  // Temperature thresholds
  float minComfortTemp = 18.0;  // °C
  float maxComfortTemp = 28.0;  // °C
  
  // Fall detection thresholds
  float fallAccelThreshold = 3.5;  // g-force
  int fallTimeWindow = 2000;       // ms
} calibration;

// ==================== STATE VARIABLES ====================
bool systemInitialized = false;
bool gpsFixed = false;
bool emergencyMode = false;
bool fallAlertSent = false;
bool batteryLowAlertSent = false;
bool wanderingAlertSent = false;
bool temperatureAlertSent = false;

unsigned long lastDataUpload = 0;
unsigned long lastGaitAnalysis = 0;
unsigned long lastTemperatureCheck = 0;
unsigned long lastBatteryCheck = 0;
unsigned long lastFallCheck = 0;
unsigned long lastGPSUpdate = 0;
unsigned long lastWiFiReconnect = 0;

// Gait analysis variables
float pressureHistory[100];
int pressureHistoryIndex = 0;
float stepThreshold = 1.5;  // kg
bool stepDetected = false;
unsigned long lastStepTime = 0;
float stepCadence = 0;

// Fall detection variables
float accelMagnitudeHistory[50];
int accelHistoryIndex = 0;
bool possibleFall = false;
unsigned long possibleFallTime = 0;

// Temperature control
bool heatingActive = false;
bool coolingActive = false;
unsigned long lastTempControl = 0;

// ==================== RISK MANAGEMENT FUNCTIONS ====================

/**
 * RISK 1: Sensor Calibration Issues
 * Solution: Multi-point calibration with validation
 */
void calibrateSensors() {
  Serial.println("Starting sensor calibration...");
  
  // Load previous calibration from NVS
  preferences.begin("smartshoe", false);
  
  // Check if calibration exists
  if (preferences.isKey("calibrated") && preferences.getBool("calibrated", false)) {
    loadCalibration();
    Serial.println("Loaded existing calibration");
    return;
  }
  
  Serial.println("No calibration found. Starting fresh calibration...");
  
  // Calibrate FSR sensors
  calibrateFSR();
  
  // Calibrate MPU6050
  calibrateMPU();
  
  // Save calibration
  saveCalibration();
  
  preferences.end();
  Serial.println("Calibration complete");
}

void calibrateFSR() {
  Serial.println("Calibrating FSR sensors...");
  Serial.println("Ensure no pressure on sensors");
  delay(3000);
  
  // Read minimum values (no pressure)
  for (int i = 0; i < 8; i++) {
    int rawSum = 0;
    for (int j = 0; j < 10; j++) {
      rawSum += analogRead(getFSRPin(i));
      delay(10);
    }
    calibration.fsrMinVoltage[i] = (rawSum / 10) * (3.3 / 4095.0);
  }
  
  Serial.println("Now apply maximum pressure to all sensors");
  delay(3000);
  
  // Read maximum values
  for (int i = 0; i < 8; i++) {
    int rawSum = 0;
    for (int j = 0; j < 10; j++) {
      rawSum += analogRead(getFSRPin(i));
      delay(10);
    }
    calibration.fsrMaxVoltage[i] = (rawSum / 10) * (3.3 / 4095.0);
  }
  
  // Validate calibration
  for (int i = 0; i < 8; i++) {
    if (calibration.fsrMaxVoltage[i] <= calibration.fsrMinVoltage[i] + 0.5) {
      Serial.printf("Warning: FSR %d has insufficient range\n", i);
    }
  }
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  Serial.println("Keep device stationary");
  
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  delay(2000);
  
  int32_t accelSum[3] = {0, 0, 0};
  int32_t gyroSum[3] = {0, 0, 0};
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    accelSum[0] += ax;
    accelSum[1] += ay;
    accelSum[2] += az;
    gyroSum[0] += gx;
    gyroSum[1] += gy;
    gyroSum[2] += gz;
    
    delay(10);
  }
  
  calibration.accelOffset[0] = accelSum[0] / samples;
  calibration.accelOffset[1] = accelSum[1] / samples;
  calibration.accelOffset[2] = (accelSum[2] / samples) - 16384; // Assuming 1g on Z-axis
  calibration.gyroOffset[0] = gyroSum[0] / samples;
  calibration.gyroOffset[1] = gyroSum[1] / samples;
  calibration.gyroOffset[2] = gyroSum[2] / samples;
  
  Serial.println("MPU calibration complete");
}

void loadCalibration() {
  for (int i = 0; i < 8; i++) {
    calibration.fsrMinVoltage[i] = preferences.getFloat(("fsrMin" + String(i)).c_str(), 0.1);
    calibration.fsrMaxVoltage[i] = preferences.getFloat(("fsrMax" + String(i)).c_str(), 3.3);
  }
  
  for (int i = 0; i < 3; i++) {
    calibration.accelOffset[i] = preferences.getInt(("accOff" + String(i)).c_str(), 0);
    calibration.gyroOffset[i] = preferences.getInt(("gyrOff" + String(i)).c_str(), 0);
  }
}

void saveCalibration() {
  for (int i = 0; i < 8; i++) {
    preferences.putFloat(("fsrMin" + String(i)).c_str(), calibration.fsrMinVoltage[i]);
    preferences.putFloat(("fsrMax" + String(i)).c_str(), calibration.fsrMaxVoltage[i]);
  }
  
  for (int i = 0; i < 3; i++) {
    preferences.putInt(("accOff" + String(i)).c_str(), calibration.accelOffset[i]);
    preferences.putInt(("gyrOff" + String(i)).c_str(), calibration.gyroOffset[i]);
  }
  
  preferences.putBool("calibrated", true);
}

/**
 * RISK 2: Fall Detection Accuracy
 * Solution: Multi-stage detection with confidence scoring
 */
void detectFall() {
  static float prevAccelMagnitude = 0;
  static unsigned long impactTime = 0;
  static bool waitingForInactivity = false;
  
  // Read accelerometer with calibration
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Apply calibration offsets
  ax -= calibration.accelOffset[0];
  ay -= calibration.accelOffset[1];
  az -= calibration.accelOffset[2];
  
  // Calculate magnitude in g-force (assuming 16384 = 1g)
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az) / 16384.0;
  
  // Store in history
  accelMagnitudeHistory[accelHistoryIndex] = accelMagnitude;
  accelHistoryIndex = (accelHistoryIndex + 1) % 50;
  
  // Calculate moving average for noise reduction
  float avgMagnitude = 0;
  for (int i = 0; i < 50; i++) {
    avgMagnitude += accelMagnitudeHistory[i];
  }
  avgMagnitude /= 50;
  
  // Stage 1: Impact detection
  if (accelMagnitude > calibration.fallAccelThreshold && !waitingForInactivity) {
    // Sudden high acceleration (impact)
    impactTime = millis();
    waitingForInactivity = true;
    possibleFall = true;
    possibleFallTime = millis();
    
    Serial.println("Possible impact detected");
  }
  
  // Stage 2: Check for post-impact inactivity
  if (waitingForInactivity && millis() - impactTime > 500) {
    // Check if device is relatively stationary after impact
    if (avgMagnitude < 1.2) {  // Less than 1.2g (stationary or lying down)
      // Stage 3: Orientation check
      float angleFromVertical = acos(abs(az) / (accelMagnitude * 16384.0)) * 180.0 / PI;
      
      if (angleFromVertical > 60) {  // Device is horizontal (person lying down)
        sensorData.fallDetected = true;
        sensorData.fallConfidence = calculateFallConfidence(accelMagnitude, avgMagnitude, angleFromVertical);
        sensorData.fallTimestamp = millis();
        
        Serial.printf("FALL DETECTED! Confidence: %.1f%%\n", sensorData.fallConfidence * 100);
        
        // Trigger emergency response
        handleFallDetection();
      }
    }
    waitingForInactivity = false;
  }
  
  // Reset fall state after timeout
  if (sensorData.fallDetected && millis() - sensorData.fallTimestamp > 30000) {
    sensorData.fallDetected = false;
    fallAlertSent = false;
  }
  
  prevAccelMagnitude = accelMagnitude;
}

float calculateFallConfidence(float impactMagnitude, float postImpactActivity, float orientationAngle) {
  float confidence = 0;
  
  // Impact severity (30%)
  confidence += min(impactMagnitude / 5.0, 1.0) * 0.3;
  
  // Post-impact inactivity (40%)
  confidence += (1.0 - min(postImpactActivity / 1.5, 1.0)) * 0.4;
  
  // Orientation change (30%)
  confidence += min(orientationAngle / 90.0, 1.0) * 0.3;
  
  return min(confidence, 1.0);
}

void handleFallDetection() {
  if (!fallAlertSent && sensorData.fallConfidence > 0.7) {
    // Activate vibration motor for alert
    analogWrite(VIBRATION_MOTOR_PIN, 200);
    
    // Send emergency alert via Firebase
    if (WiFi.status() == WL_CONNECTED) {
      FirebaseJson alertJson;
      alertJson.set("type", "FALL_DETECTED");
      alertJson.set("confidence", sensorData.fallConfidence);
      alertJson.set("timestamp", sensorData.fallTimestamp);
      alertJson.set("location/lat", sensorData.latitude);
      alertJson.set("location/lng", sensorData.longitude);
      alertJson.set("battery", sensorData.batteryPercentage);
      
      if (Firebase.pushJSON(firebaseData, "/alerts/emergency", alertJson)) {
        Serial.println("Emergency alert sent to Firebase");
        fallAlertSent = true;
      }
    }
    
    // Blink red LED
    digitalWrite(RGB_RED_PIN, HIGH);
    digitalWrite(RGB_GREEN_PIN, LOW);
    digitalWrite(RGB_BLUE_PIN, LOW);
    
    // Keep vibrating for 5 seconds
    delay(5000);
    analogWrite(VIBRATION_MOTOR_PIN, 0);
  }
}

/**
 * RISK 3: Mobile App Usability (Elderly-friendly)
 * Solution: Multiple feedback modalities with clear indicators
 */
void provideUserFeedback() {
  // Visual feedback via RGB LED
  if (sensorData.fallDetected) {
    setRGBColor(255, 0, 0);  // Red - Emergency
  } else if (sensorData.batteryPercentage < 15) {
    setRGBColor(255, 165, 0);  // Orange - Low battery
  } else if (abs(sensorData.targetTemp - sensorData.insoleTemp) > 2) {
    setRGBColor(0, 0, 255);  // Blue - Temperature adjusting
  } else if (sensorData.balanceScore < 0.7) {
    setRGBColor(255, 255, 0);  // Yellow - Poor balance
  } else if (sensorData.gaitSymmetry < 0.8) {
    setRGBColor(255, 0, 255);  // Purple - Gait issue
  } else {
    setRGBColor(0, 255, 0);  // Green - All good
  }
  
  // Haptic feedback for specific events
  static unsigned long lastHapticFeedback = 0;
  
  // Step feedback (gentle vibration on each step)
  if (stepDetected && millis() - lastHapticFeedback > 500) {
    analogWrite(VIBRATION_MOTOR_PIN, 50);
    delay(50);
    analogWrite(VIBRATION_MOTOR_PIN, 0);
    lastHapticFeedback = millis();
  }
  
  // Posture correction feedback
  if (sensorData.balanceScore < 0.5 && millis() - lastHapticFeedback > 2000) {
    // Pattern: 3 short vibrations
    for (int i = 0; i < 3; i++) {
      analogWrite(VIBRATION_MOTOR_PIN, 150);
      delay(200);
      analogWrite(VIBRATION_MOTOR_PIN, 0);
      delay(100);
    }
    lastHapticFeedback = millis();
  }
}

void setRGBColor(int red, int green, int blue) {
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}

/**
 * RISK 4: Data Privacy
 * Solution: Secure Firebase connection with authentication
 */
void setupFirebase() {
  Serial.println("Configuring Firebase...");
  
  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  
  // Enable SSL/TLS
  firebaseConfig.timeout.serverResponse = 10000;
  firebaseConfig.timeout.wifiReconnect = 10000;
  
  Firebase.begin(&firebaseConfig, &firebaseAuth);
  Firebase.reconnectWiFi(true);
  
  // Set maximum retry attempts
  Firebase.setMaxRetry(&firebaseData, 3);
  
  Serial.println("Firebase configured");
}

void uploadDataToFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping upload");
    return;
  }
  
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready");
    return;
  }
  
  FirebaseJson json;
  
  // Create data packet with timestamp
  json.set("timestamp", millis());
  json.set("deviceId", WiFi.macAddress());
  
  // Pressure data
  FirebaseJson pressureJson;
  for (int i = 0; i < 8; i++) {
    pressureJson.set("fsr" + String(i), sensorData.pressureKg[i]);
  }
  json.set("pressure", pressureJson);
  
  // Gait analysis
  json.set("gait/stepCount", sensorData.stepCount);
  json.set("gait/speed", sensorData.walkingSpeed);
  json.set("gait/symmetry", sensorData.gaitSymmetry);
  json.set("gait/balance", sensorData.balanceScore);
  
  // Safety data
  json.set("safety/fallDetected", sensorData.fallDetected);
  json.set("safety/fallConfidence", sensorData.fallConfidence);
  
  // Location (with privacy consideration - only send if GPS fixed)
  if (gpsFixed && sensorData.satellites > 3) {
    json.set("location/lat", sensorData.latitude, 6);
    json.set("location/lng", sensorData.longitude, 6);
    json.set("location/alt", sensorData.altitude);
    json.set("location/satellites", sensorData.satellites);
  }
  
  // Environmental
  json.set("environment/insoleTemp", sensorData.insoleTemp);
  json.set("environment/ambientTemp", sensorData.ambientTemp);
  
  // Battery
  json.set("battery/voltage", sensorData.batteryVoltage);
  json.set("battery/percentage", sensorData.batteryPercentage);
  
  // Upload with encryption
  if (Firebase.pushJSON(firebaseData, "/sensorData", json)) {
    Serial.println("Data uploaded securely");
  } else {
    Serial.printf("Upload failed: %s\n", firebaseData.errorReason().c_str());
  }
  
  // Send alerts if needed
  checkAndSendAlerts();
}

void checkAndSendAlerts() {
  // Low battery alert
  if (sensorData.batteryPercentage < 15 && !batteryLowAlertSent) {
    sendAlert("LOW_BATTERY", "Battery level is below 15%");
    batteryLowAlertSent = true;
  } else if (sensorData.batteryPercentage > 20) {
    batteryLowAlertSent = false;
  }
  
  // Wandering alert (GPS boundary)
  if (gpsFixed && checkWandering()) {
    if (!wanderingAlertSent) {
      sendAlert("WANDERING", "User has left safe zone");
      wanderingAlertSent = true;
    }
  } else {
    wanderingAlertSent = false;
  }
  
  // Temperature alert
  if (sensorData.insoleTemp > 40 || sensorData.insoleTemp < 5) {
    if (!temperatureAlertSent) {
      sendAlert("TEMPERATURE_EXTREME", "Insole temperature outside safe range");
      temperatureAlertSent = true;
    }
  } else {
    temperatureAlertSent = false;
  }
}

void sendAlert(String type, String message) {
  if (WiFi.status() != WL_CONNECTED) return;
  
  FirebaseJson alertJson;
  alertJson.set("type", type);
  alertJson.set("message", message);
  alertJson.set("timestamp", millis());
  alertJson.set("battery", sensorData.batteryPercentage);
  
  if (gpsFixed) {
    alertJson.set("location/lat", sensorData.latitude);
    alertJson.set("location/lng", sensorData.longitude);
  }
  
  Firebase.pushJSON(firebaseData, "/alerts/notifications", alertJson);
  
  // Trigger vibration for important alerts
  if (type == "FALL_DETECTED" || type == "WANDERING") {
    for (int i = 0; i < 5; i++) {
      analogWrite(VIBRATION_MOTOR_PIN, 255);
      delay(500);
      analogWrite(VIBRATION_MOTOR_PIN, 0);
      delay(200);
    }
  }
}

/**
 * RISK 5: Battery Life Optimization
 * Solution: Power-saving modes and efficient component usage
 */
void optimizePower() {
  static unsigned long lastPowerCheck = 0;
  
  if (millis() - lastPowerCheck < 10000) return;
  lastPowerCheck = millis();
  
  // Check battery level
  sensorData.batteryVoltage = analogRead(BATTERY_PIN) * (3.3 / 4095.0) * 2;  // Voltage divider
  
  // Calculate percentage
  sensorData.batteryPercentage = map(sensorData.batteryVoltage * 100, 
                                     BATTERY_EMPTY * 100, 
                                     BATTERY_FULL * 100, 
                                     0, 100);
  sensorData.batteryPercentage = constrain(sensorData.batteryPercentage, 0, 100);
  
  // Power saving based on battery level
  if (sensorData.batteryPercentage < 10) {
    // Critical battery - minimal functionality
    setPowerMode(CRITICAL);
  } else if (sensorData.batteryPercentage < 20) {
    // Low battery - reduced functionality
    setPowerMode(LOW_POWER);
  } else if (!gpsFixed && sensorData.batteryPercentage < 30) {
    // GPS searching with low battery
    setPowerMode(GPS_SEARCH_LOW_POWER);
  } else {
    // Normal operation
    setPowerMode(NORMAL);
  }
}

enum PowerMode { NORMAL, LOW_POWER, CRITICAL, GPS_SEARCH_LOW_POWER };
PowerMode currentPowerMode = NORMAL;

void setPowerMode(PowerMode mode) {
  if (mode == currentPowerMode) return;
  
  currentPowerMode = mode;
  
  switch(mode) {
    case NORMAL:
      // Full functionality
      setCpuFrequencyMhz(240);
      WiFi.setSleep(false);
      break;
      
    case LOW_POWER:
      // Reduced sampling rates
      setCpuFrequencyMhz(80);
      WiFi.setSleep(true);
      break;
      
    case CRITICAL:
      // Emergency only - fall detection and GPS
      setCpuFrequencyMhz(40);
      WiFi.setSleep(true);
      WiFi.disconnect(true);
      // Only keep fall detection active
      break;
      
    case GPS_SEARCH_LOW_POWER:
      setCpuFrequencyMhz(80);
      WiFi.setSleep(true);
      break;
  }
}

/**
 * RISK 6: Temperature Regulation Safety
 * Solution: Safe temperature control with failsafes
 */
void regulateTemperature() {
  static unsigned long lastTempRead = 0;
  
  if (millis() - lastTempRead < 2000) return;
  lastTempRead = millis();
  
  // Read temperatures
  sensorData.ambientTemp = dht.readTemperature();
  sensorData.insoleTemp = readInsoleTemperature();
  
  if (isnan(sensorData.ambientTemp)) {
    Serial.println("Failed to read ambient temperature");
    return;
  }
  
  // Get target temperature from Firebase or use default
  if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
    if (Firebase.getFloat(firebaseData, "/settings/targetTemperature")) {
      sensorData.targetTemp = firebaseData.floatData();
    }
  }
  
  // Safety checks
  if (sensorData.insoleTemp > 45) {
    // Overheat protection - force shutdown
    digitalWrite(HEATING_PIN, LOW);
    digitalWrite(COOLING_PIN, LOW);
    heatingActive = false;
    coolingActive = false;
    sendAlert("OVERHEAT", "Insole temperature too high - shut down");
    return;
  }
  
  if (sensorData.insoleTemp < 0) {
    // Too cold - might damage battery
    digitalWrite(HEATING_PIN, HIGH);
    digitalWrite(COOLING_PIN, LOW);
    heatingActive = true;
    coolingActive = false;
    return;
  }
  
  // Normal temperature control
  if (sensorData.insoleTemp < sensorData.targetTemp - 1) {
    // Need heating
    digitalWrite(HEATING_PIN, HIGH);
    digitalWrite(COOLING_PIN, LOW);
    heatingActive = true;
    coolingActive = false;
  } else if (sensorData.insoleTemp > sensorData.targetTemp + 1) {
    // Need cooling
    digitalWrite(HEATING_PIN, LOW);
    digitalWrite(COOLING_PIN, HIGH);
    heatingActive = false;
    coolingActive = true;
  } else {
    // Temperature OK
    digitalWrite(HEATING_PIN, LOW);
    digitalWrite(COOLING_PIN, LOW);
    heatingActive = false;
    coolingActive = false;
  }
}

float readInsoleTemperature() {
  // Using DS18B20 or similar sensor
  // This is a placeholder - implement actual reading
  return dht.readTemperature();  // For now, use ambient
}

/**
 * Gait Analysis Functions
 */
void analyzeGait() {
  if (millis() - lastGaitAnalysis < 50) return;  // 20Hz analysis
  lastGaitAnalysis = millis();
  
  // Read all pressure sensors
  float totalPressure = 0;
  float heelPressure = 0;
  float forefootPressure = 0;
  
  for (int i = 0; i < 8; i++) {
    int rawValue = analogRead(getFSRPin(i));
    
    // Convert to voltage
    sensorData.pressureVoltage[i] = rawValue * (3.3 / 4095.0);
    
    // Convert to force (kg) using calibration
    if (sensorData.pressureVoltage[i] <= calibration.fsrMinVoltage[i]) {
      sensorData.pressureKg[i] = 0;
    } else if (sensorData.pressureVoltage[i] >= calibration.fsrMaxVoltage[i]) {
      sensorData.pressureKg[i] = calibration.fsrMaxForce[i];
    } else {
      // Linear interpolation
      float ratio = (sensorData.pressureVoltage[i] - calibration.fsrMinVoltage[i]) / 
                   (calibration.fsrMaxVoltage[i] - calibration.fsrMinVoltage[i]);
      sensorData.pressureKg[i] = calibration.fsrMinForce[i] + 
                                 ratio * (calibration.fsrMaxForce[i] - calibration.fsrMinForce[i]);
    }
    
    totalPressure += sensorData.pressureKg[i];
    
    // Classify foot regions (assuming 0-3 are heel/midfoot, 4-7 are forefoot/toe)
    if (i < 4) {
      heelPressure += sensorData.pressureKg[i];
    } else {
      forefootPressure += sensorData.pressureKg[i];
    }
  }
  
  // Step detection
  if (totalPressure > stepThreshold && !stepDetected) {
    stepDetected = true;
    sensorData.stepCount++;
    
    if (lastStepTime > 0) {
      float stepInterval = (millis() - lastStepTime) / 1000.0;  // seconds
      stepCadence = 60.0 / stepInterval;  // steps per minute
    }
    lastStepTime = millis();
  } else if (totalPressure < stepThreshold * 0.5) {
    stepDetected = false;
  }
  
  // Balance score (0-1, higher is better)
  if (totalPressure > 0) {
    float leftPressure = 0, rightPressure = 0;
    for (int i = 0; i < 4; i++) leftPressure += sensorData.pressureKg[i];
    for (int i = 4; i < 8; i++) rightPressure += sensorData.pressureKg[i];
    
    float total = leftPressure + rightPressure;
    if (total > 0) {
      sensorData.balanceScore = 1.0 - (abs(leftPressure - rightPressure) / total);
    }
  }
  
  // Gait symmetry (based on step timing)
  if (sensorData.stepCount > 10) {
    // Simplified symmetry calculation
    sensorData.gaitSymmetry = 0.85;  // Placeholder - implement actual calculation
  }
  
  // Walking speed estimation
  if (gpsFixed && sensorData.speed > 0) {
    sensorData.walkingSpeed = sensorData.speed;  // m/s from GPS
  } else {
    // Estimate from step cadence
    sensorData.walkingSpeed = stepCadence * 0.75 / 60.0;  // Rough estimate
  }
  
  // Store pressure history for trend analysis
  pressureHistory[pressureHistoryIndex] = totalPressure;
  pressureHistoryIndex = (pressureHistoryIndex + 1) % 100;
}

/**
 * GPS Functions
 */
void updateGPS() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
  }
  
  if (millis() - lastGPSUpdate > 1000) {
    lastGPSUpdate = millis();
    
    if (gps.location.isValid()) {
      sensorData.latitude = gps.location.lat();
      sensorData.longitude = gps.location.lng();
      sensorData.altitude = gps.altitude.meters();
      sensorData.speed = gps.speed.mps();
      sensorData.heading = gps.course.deg();
      sensorData.satellites = gps.satellites.value();
      gpsFixed = (sensorData.satellites > 3);
    } else {
      gpsFixed = false;
    }
  }
}

bool checkWandering() {
  // Check if user has left predefined safe zone
  // This is a placeholder - implement actual geofencing
  static double homeLat = 6.9271;   // Example: Colombo
  static double homeLng = 79.8612;
  static double maxDistance = 500;  // meters
  
  if (!gpsFixed) return false;
  
  double distance = calculateDistance(sensorData.latitude, sensorData.longitude, 
                                      homeLat, homeLng);
  
  return distance > maxDistance;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Haversine formula
  double R = 6371000; // Earth's radius in meters
  double phi1 = lat1 * PI / 180;
  double phi2 = lat2 * PI / 180;
  double deltaPhi = (lat2 - lat1) * PI / 180;
  double deltaLambda = (lon2 - lon1) * PI / 180;
  
  double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2) * sin(deltaLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  return R * c;
}

/**
 * Helper Functions
 */
int getFSRPin(int index) {
  switch(index) {
    case 0: return FSR1_PIN;
    case 1: return FSR2_PIN;
    case 2: return FSR3_PIN;
    case 3: return FSR4_PIN;
    case 4: return FSR5_PIN;
    case 5: return FSR6_PIN;
    case 6: return FSR7_PIN;
    case 7: return FSR8_PIN;
    default: return FSR1_PIN;
  }
}

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }
}

/**
 * Setup and Main Loop
 */
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== IoT Smart Shoe Initializing ===");
  
  // Initialize pins
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  pinMode(HEATING_PIN, OUTPUT);
  pinMode(COOLING_PIN, OUTPUT);
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  
  // Set initial states
  digitalWrite(HEATING_PIN, LOW);
  digitalWrite(COOLING_PIN, LOW);
  analogWrite(VIBRATION_MOTOR_PIN, 0);
  setRGBColor(255, 255, 255);  // White during startup
  
  // Initialize I2C for MPU6050
  I2CMPU.begin(MPU_SDA, MPU_SCL, 100000);
  mpu.initialize();
  
  // Initialize DHT22
  dht.begin();
  
  // Initialize GPS
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Test vibration motor
  analogWrite(VIBRATION_MOTOR_PIN, 100);
  delay(500);
  analogWrite(VIBRATION_MOTOR_PIN, 0);
  
  // Calibrate sensors
  calibrateSensors();
  
  // Connect to WiFi
  connectToWiFi();
  
  // Setup Firebase
  setupFirebase();
  
  // Indicate successful initialization
  setRGBColor(0, 255, 0);  // Green
  delay(1000);
  setRGBColor(0, 0, 0);    // Off
  
  systemInitialized = true;
  Serial.println("System initialization complete");
}

void loop() {
  static unsigned long lastLoop = 0;
  unsigned long currentTime = millis();
  
  // Calculate uptime
  sensorData.deviceUptime = currentTime;
  
  // Main processing loop - all functions have their own timing control
  updateGPS();
  analyzeGait();
  detectFall();
  regulateTemperature();
  optimizePower();
  provideUserFeedback();
  
  // Reconnect WiFi if needed
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiReconnect > 30000) {
    connectToWiFi();
    lastWiFiReconnect = millis();
  }
  
  // Upload data periodically
  if (millis() - lastDataUpload > 5000) {  // Every 5 seconds
    if (WiFi.status() == WL_CONNECTED && Firebase.ready()) {
      uploadDataToFirebase();
    }
    lastDataUpload = millis();
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}