/*
 * Configuration file for Smart Shoe ESP32
 * Store all sensitive data and configurable parameters here
 */

#ifndef CONFIG_H
#define CONFIG_H

// ====================== Bluetooth Configuration ======================
#define BT_DEVICE_NAME "SmartShoe_Group14"
#define BT_PIN_CODE "1234"  // Optional pairing code

// ====================== Sensor Thresholds ======================
// Fall detection (in mg units from MPU6050)
#define FALL_IMPACT_THRESHOLD   2500  // Sudden impact threshold
#define FALL_INACTIVITY_THRESHOLD 200 // Post-fall inactivity threshold (mg)
#define FALL_CONFIRM_TIME       2000  // Time to confirm fall (ms)

// Pressure sensor thresholds
#define PRESSURE_MIN             10   // Minimum pressure to register
#define PRESSURE_MAX             4095 // Maximum analog reading
#define IMBALANCE_THRESHOLD_PCT  30   // Imbalance percentage threshold

// Step detection
#define STEP_HIGH_THRESHOLD      18000 // Heel strike threshold
#define STEP_LOW_THRESHOLD       14000 // Toe-off threshold
#define STEP_DEBOUNCE_TIME       300   // Minimum time between steps (ms)

// Battery monitoring
#define BATTERY_FULL_VOLTAGE     4.2   // Fully charged voltage
#define BATTERY_EMPTY_VOLTAGE    3.3   // Empty voltage
#define BATTERY_LOW_VOLTAGE      3.4   // Low battery warning threshold

// ====================== Timing Configuration ======================
// All times in milliseconds
#define SENSOR_READ_INTERVAL     100   // Read sensors every 100ms (10Hz)
#define GPS_READ_INTERVAL        1000  // Update GPS every 1 second
#define DATA_SEND_INTERVAL       500   // Send data via BT every 500ms
#define BATTERY_CHECK_INTERVAL   60000 // Check battery every minute

// ====================== Pin Mapping ======================
// Left Shoe FSRs
#define PIN_FSR_LEFT_HEEL    34
#define PIN_FSR_LEFT_MID     35
#define PIN_FSR_LEFT_BALL    32
#define PIN_FSR_LEFT_TOE     33

// Right Shoe FSRs
#define PIN_FSR_RIGHT_HEEL   36
#define PIN_FSR_RIGHT_MID    39
#define PIN_FSR_RIGHT_BALL    34  // Share pins if needed (multiplexing)
#define PIN_FSR_RIGHT_TOE     35  // Or use I2C ADC expander

// Outputs
#define PIN_VIBRATION_MOTOR  25
#define PIN_LED_CONNECTION    2
#define PIN_LED_CHARGING      4
#define PIN_LED_ERROR        16

// Sensors
#define PIN_TEMP_SENSOR      26
#define PIN_BATTERY          27

// ====================== Debug Configuration ======================
#define DEBUG_ENABLED true
#define DEBUG_BAUD_RATE 115200

// ====================== Feature Flags ======================
#define ENABLE_GPS true
#define ENABLE_TEMPERATURE_CONTROL true
#define ENABLE_FALL_DETECTION true
#define ENABLE_STEP_COUNTING true
#define ENABLE_HAPTIC_FEEDBACK true

#endif // CONFIG_H
