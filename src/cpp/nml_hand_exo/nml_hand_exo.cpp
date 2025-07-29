/**
 * @file nml_hand_exo.cpp
 * @brief Implementation file for the NML Hand Exoskeleton API.
 *
 * This file contains the implementation of the NMLHandExo class, which handles
 * initialization, motor control, angle management, and device telemetry for the
 * exoskeleton.
 */
#include "config.h"
//#include "utils.h"
#include "nml_hand_exo.h"
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.

/// @brief Serial port for Dynamixel communication.
/// @brief Pin assignment for the Dynamixel direction control pin.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  //#include <SoftwareSerial.h>
  //SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial2
  //#define BLE_SERIAL soft_serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define COMMAND_SERIAL Serial1
  //#define DEBUG_SERIAL Serial1
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  #define COMMAND_SERIAL Serial2
  //#define DEBUG_SERIAL SerialUSB
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DEBUG_SERIAL Serial
  #define COMMAND_SERIAL Serial2
  #define DXL_SERIAL   Serial3
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL Serial1
  #define COMMAND_SERIAL Serial2
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DEBUG_SERIAL Serial
  #define DXL_SERIAL   Serial1
  //#define DEBUG_SERIAL Serial
  //const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

/// @brief Verbose output toggle for debugging.
bool VERBOSE = DEFAULT_VERBOSE; // default to true

/// @brief Flag for mode switching
static volatile bool modeSwitchFlag = false;

/// @brief Checking last interrupt time to check for unwanted presses, control debounce
static unsigned long lastInterruptTime = 0;

Stream* debugStream = &DEBUG_SERIAL;

/// @brief Mode press function
void onModeButtonPress() {
  debugPrint(F("Button pressed"));
  unsigned long now = millis();
  if (now - lastInterruptTime > BUTTON_DEBOUNCE_DURATION) {  // debounce
    digitalWrite(LED_BUILTIN, HIGH);
    modeSwitchFlag = true;
    lastInterruptTime = now;
  }
}

NMLHandExo::NMLHandExo(const uint8_t* ids, uint8_t numMotors, const float jointLimits[][2], const float* homeState)
  : dxl_(DXL_SERIAL, DXL_DIR_PIN), motorIds_(ids), numMotors_(numMotors) //jointLimits_(jointLimits),
{

  // Allocate and copy joint limits
  jointLimits_ = new float[numMotors_][2];
  for (int i = 0; i < numMotors_; ++i) {
    jointLimits_[i][0] = jointLimits[i][0]; // min
    jointLimits_[i][1] = jointLimits[i][1]; // max
  }

  zeroOffsets_ = new float[numMotors_];

  // Create offsets if values passed for homeState
  if (homeState != nullptr) {
    for (int i = 0; i < numMotors_; ++i) {
      zeroOffsets_[i] = homeState[i];
    }
  } else {
    // Default all to 0.0
    for (int i = 0; i < numMotors_; ++i) {
      zeroOffsets_[i] = 0.0f;
    }
  }

  // Allocate and initialize current limits
  currentLimits_ = new uint16_t[numMotors_];
  for (int i = 0; i < numMotors_; ++i) {
      currentLimits_[i] = MOTOR_CURRENT_LIMIT; // default 200 mA or whatever safe default
  }

  // If jointLimits_, zeroOffsets_, currentLimits_ were dynamically allocated, make sure to add a destructor.
}
// ====================================================================================
// ================================ Utility functions =================================
// ====================================================================================
void NMLHandExo::initializeSerial(int baud) {
  // Initialize serial communication with DYNAMIXEL hardware using the specified baudrate. Has to match hardware
  dxl_.begin(baud);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
void NMLHandExo::initializeMotors() {
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    dxl_.torqueOff(id);

    // Set Operating Mode to Current-Based Position Control
    //dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    //dxl_.writeControlTableItem(ControlTableItem::OPERATING_MODE, id, 5);
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    //dxl_.setOperatingMode(id, OP_POSITION);  // Default mode is set to position mode

    dxl_.torqueOn(id);

    //dxl_.writeControlTableItem(ControlTableItem::GOAL_CURRENT, currentLimits_[i], 100);
    dxl_.setGoalCurrent(id, currentLimits_[i]);
  }
  delay(100); // Allow time for motors to initialize
}
int NMLHandExo::getMotorID(const String& token) {
  String target = token;
  target.trim();
  target.toUpperCase();
  int id = target.toInt(); // Try converting to integer

  // Check if it was a valid number (e.g., not "WRIST")
  if (id != 0 || target == "0") {
    return id;
  }
  return getMotorIDByName(target);  // Otherwise, try name lookup
}
uint8_t NMLHandExo::getMotorIDByIndex(const int index) {
  return motorIds_[index];
}
int NMLHandExo::getIndexById(uint8_t id) {
  for (int i = 0; i < numMotors_; i++) {
    if (motorIds_[i] == id) return i;
  }
  return -1;
}
int NMLHandExo::getMotorIDByName(const String& name) {
  String n = name;
  n.toUpperCase();
  if (n == "WRIST") return 1;
  if (n == "THUMB") return 2;
  if (n == "INDEX") return 3;
  if (n == "MIDDLE") return 4;
  if (n == "RING") return 5;
  if (n == "PINKY") return 6;
  return -1;
}
//const char* getMotorName(int index) {
//    return motorNames_[index];  // assuming motorNames_ is stored
//}
void NMLHandExo::setMotorNames(const char* const* names) {
  motorNames_ = names;
}
String NMLHandExo::getMotorNameByID(uint8_t id) {
  for (int i = 0; i < numMotors_; ++i) {
    if (motorIds_[i] == id) {
      return motorNames_ ? String(motorNames_[i]) : "unnamed";
    }
  }
  return "unknown";
}
int NMLHandExo::angleToTicks(float angle_deg, int index) {
  // Map degrees to ticks: assume full range = 4096 ticks = 360 deg
  float deg_per_tick = 300.0 / PULSE_RESOLUTION;
  int ticks = static_cast<int>(angle_deg / deg_per_tick);
  return ticks;
}
void NMLHandExo::setZeroOffset(uint8_t id) {
  int index = getIndexById(id);
  if (index != -1) {
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[index] = current_angle;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Calibrated zero for motor %d: %f deg", id, current_angle);
    debugPrint(buffer);
    //debugPrint("Calibrated zero for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  } else {
    debugPrint("[ERROR] Invalid motor ID for zero calibration: " + String(id));
  }
}
float NMLHandExo::getZeroOffset(uint8_t id) {
  int index = getIndexById(id);
  return (index != -1) ? zeroOffsets_[index] : 0.0f;
}
void NMLHandExo::resetAllZeros() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    float current_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
    zeroOffsets_[i] = current_angle;
    debugPrint("[DEBUG] Zero offset set for motor " + String(id) + ": " + String(current_angle, 2) + " deg");
  }
}
String NMLHandExo::getDeviceInfo() {

    // Need to return a single string with all the information
    String info = "Name: NMLHandExo;\n";
    info += "Version: " + String(VERSION) + ";\n";
    info += "Number of Motors: " + String(numMotors_) + ";\n";
    for (int i = 0; i < numMotors_; ++i) {
      const char* name = getMotorNameByID(motorIds_[i]).c_str();
      float angle = getRelativeAngle(motorIds_[i]);
      float minLimit = jointLimits_[i][0];
      float maxLimit = jointLimits_[i][1];
      float torque = getTorque(motorIds_[i]);
      bool isEnabled = getTorqueEnabledStatus(motorIds_[i]);

      info += "Motor " + String(i) + ": {name: " + String(name) +
            ", id: " + String(motorIds_[i]) +
            ", angle: " + String(angle, 2) +
            ", limits: [" + String(minLimit, 2) + ", " + String(maxLimit, 2) + "]" +
            ", torque: " + String(torque, 2) +
            ", enabled: " + (isEnabled ? "true" : "false") + "};\n";
      }
    return info;
}

int NMLHandExo::getMotorCount() {
  return numMotors_;
}

// ====================================================================================
// ============================ Calibration commands ==================================
// ====================================================================================
void NMLHandExo::beginCalibration(bool enableTimedCalibration=false, int duration=10) {
  isCalibrating = true;
  calibrationTimedMode = enableTimedCalibration;
  calibrationStartTime = millis();
  calibrationDuration = duration * 1000;

  // Initialize calibration state and ask user to move their fingers to extremes. The calibration process will last as long as the calibration duration in seconds
  // This is done only if timedCalibration is true. If off, step through the calibration process starting with asking the user to close their fingers, followed by opening their fingers
  if (calibrationTimedMode) {
      DEBUG_SERIAL.println("[Gesture] Timed calibration mode. You have " + String(duration) + " seconds to complete.");
  } else {
      DEBUG_SERIAL.println(F("[Gesture] Step-through calibration mode. Follow prompts to set limits."));
  }
  for (uint8_t i = 0; i < numMotors_; ++i) {
    jointLimits_[i][0] = 1e6; // Initialize to a very large value
    jointLimits_[i][1] = -1e6; // Initialize to a very small value
  }
  debugPrint(F("[Exo Calibration] Started. Move all motors to full range."));
}
void NMLHandExo::updateCalibration() {
  if (!isCalibrating) return;

  unsigned long currentTime = millis();
  float elapsedSec = (currentTime - calibrationStartTime) / 1000.0f;

  // Update joint limits
  for (uint8_t i = 0; i < numMotors_; ++i) {
    float angle = getAbsoluteAngle(i);
    if (angle < jointLimits_[i][0]) jointLimits_[i][0] = angle;
    if (angle > jointLimits_[i][1]) jointLimits_[i][1] = angle;
  }

  // Stop condition
  if ((calibrationTimedMode && elapsedSec >= calibrationDuration*1000) || (!calibrationTimedMode && this->checkModeSwitchButtonPressed())) {
    isCalibrating = false;
    DEBUG_SERIAL.println("[Gesture] Calibration complete.");
    for (uint8_t i = 0; i < this->getMotorCount(); ++i) {
      DEBUG_SERIAL.println("Motor " + String(i) + ": Min = " + String(jointLimits_[i][0]) +
                           ", Max = " + String(jointLimits_[i][1]));
    }

    // TO-DO: Optionally store or update gesture thresholds here, possibly
  }
}
bool NMLHandExo::isExoCalibrating() {
  // Check if the exoskeleton is currently in calibration mode
  return isCalibrating;
}

// ====================================================================================
// ================================= Mode commands ====================================
// ====================================================================================
void NMLHandExo::update() {

    // First check if we are calibrating
    if (isExoCalibrating()) {
        updateCalibration();
        return; // Skip other updates while calibrating
    }

    // Check for button pushes
    if (checkModeSwitchButtonPressed()) {
        debugPrint("Mode switch button pressed");
        String exo_mode = getExoOperatingMode();
        cycleExoOperatingMode();
        // if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
        //     // === Button was pressed ===
        //     debugPrint(F("[HandExo] Button press detected, cycling exo mode."));
        //     cycleExoOperatingMode();
        // } else {
        //     debugPrint(F("[HandExo] Button press detected, but exo is in FREE mode. No action taken."));
        // }
    }
}
void NMLHandExo::setModeSwitchButton(int pin) {
  modeSwitchPin = pin;
  pinMode(modeSwitchPin, INPUT_PULLUP);

  lastButtonState = HIGH;
  buttonState = HIGH;
  lastDebounceTime = 0;

  char buffer[64];
  debugPrint("Mode switch button set on pin " + String(modeSwitchPin));
}
void NMLHandExo::setExoOperatingMode(const String& modeStr) {
  String m = modeStr;
  m.toUpperCase();

  if (m == "FREE") {
    exoMode_ = FREE;
  } else if (m == "GESTURE_FIXED") {
    exoMode_ = GESTURE_FIXED;
  } else if (m == "GESTURE_CONTINUOUS") {
    exoMode_ = GESTURE_CONTINUOUS;
  } else {
    debugPrint(F("[ERROR] Invalid EXO mode passed"));
  }
  debugPrint("Exo mode set to: " + m);
}
ExoOperatingMode NMLHandExo::getExoOperatingModeEnum() {
  // Return the current operating mode of the exoskeleton as an enum
  return exoMode_;
}
String NMLHandExo::getExoOperatingMode() {
  // Return the current operating mode of the exoskeleton
  ExoOperatingMode mode = getExoOperatingModeEnum();
  switch (mode) {
      case FREE:
      return "FREE";
      case GESTURE_FIXED:
      return "GESTURE_FIXED";
      case GESTURE_CONTINUOUS:
      return "GESTURE_CONTINUOUS";
      default:
      return "UNKNOWN";
  }
}
bool NMLHandExo::checkModeSwitchButtonPressed() {
  if (modeSwitchPin == -1) return false;
  int reading = digitalRead(modeSwitchPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        // === Button was pressed ===
        return true;
      }
    }
  }
  lastButtonState = reading;
  return false;
}
void NMLHandExo::cycleExoOperatingMode() {
  exoMode_ = static_cast<ExoOperatingMode>((exoMode_ + 1) % 3);  // cycles 0–2
  switch (exoMode_) {
    case FREE:
      debugPrint(F("Mode changed to: FREE"));
      break;
    case GESTURE_FIXED:
      debugPrint(F("Mode changed to: GESTURE_FIXED"));
      break;
    case GESTURE_CONTINUOUS:
      debugPrint(F("Mode changed to: GESTURE_CONTINUOUS"));
      break;
  }
}



// ====================================================================================
// ============================== Position commands ====================================
// ====================================================================================
float NMLHandExo::getRelativeAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) return -1;

  float abs_angle = dxl_.getPresentPosition(id, UNIT_DEGREE);
  float rel_angle = abs_angle - zeroOffsets_[index];
  return rel_angle;
}
void NMLHandExo::setRelativeAngle(uint8_t id, float relativeAngle) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return;
  }

  // Compute the absolute angle by adding the stored offset
  float abs_goal = zeroOffsets_[index] + relativeAngle;

  // Clamp the absolute goal to the joint limits (if necessary)
  abs_goal = constrain(abs_goal, jointLimits_[index][0], jointLimits_[index][1]);

  // Command the motor to the absolute position
  dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE);
  //dxl_.writeControlTableItem(ControlTableItem::GOAL_POSITION, id, abs_goal_ticks);

  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Motor %d set to relative angle %.2f deg (absolute: %.2f deg)", id, relativeAngle, abs_goal);
  debugPrint(buffer);
}
float NMLHandExo::getAbsoluteAngle(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return -1;
  }
  return dxl_.getPresentPosition(id, UNIT_DEGREE);
}
void NMLHandExo::setAbsoluteAngle(uint8_t id, float absoluteAngle) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID: " + String(id));
    return;
  }
  dxl_.setGoalPosition(id, absoluteAngle, UNIT_DEGREE);
  //char buffer[64];
  //snprintf(buffer, sizeof(buffer), "Setting motor %d to absolute angle %.2f", id, absoluteAngle);
  //debugPrint(buffer);
  debugPrint("Setting motor " + String(id) + " to absolute angle " + String(absoluteAngle, 2));
}
float NMLHandExo::getZeroAngle(uint8_t id){
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint(F("Invalid motor ID"));
    return -1;
  }

  return zeroOffsets_[index];
}
void NMLHandExo::setHome(uint8_t id){
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint(F("Invalid motor ID"));
    return;
  }

  // Command the motor to move to the stored zero offset position
  float homeAngle = zeroOffsets_[index];
  dxl_.setGoalPosition(id, homeAngle, UNIT_DEGREE);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Motor %d homing to %.2f deg", id, homeAngle);
  debugPrint(buffer);
}
void NMLHandExo::homeAllMotors() {
  for (int i = 0; i < numMotors_; ++i) {
    uint8_t id = motorIds_[i];
    setHome(id);
  }
}
void NMLHandExo::setAngleById(uint8_t id, float angle_deg) {
  int index = getIndexById(id);
  if (index == -1) return;

  // Apply offset to relative angle position
  float abs_goal = zeroOffsets_[index] + angle_deg;

  // Clamp angle to joint limits (in degrees)
  abs_goal = constrain(abs_goal, jointLimits_[index][0], jointLimits_[index][1]);

  // Set new goal tick position
  dxl_.setGoalPosition(id, abs_goal, UNIT_DEGREE);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Setting motor %d to angle %.2f deg", id, abs_goal);
  debugPrint(buffer);
}
void NMLHandExo::setAngleByAlias(const String& alias, float angleDeg) {
  String name = alias;
  name.toUpperCase();
  if (name == "WRIST") setAngleById(1, angleDeg);
  else if (name == "THUMB") setAngleById(2, angleDeg);
  else if (name == "INDEX") setAngleById(3, angleDeg);
  else if (name == "MIDDLE") setAngleById(4, angleDeg);
  else if (name == "RING") setAngleById(5, angleDeg);
  else if (name == "PINKY") setAngleById(6, angleDeg);
}
void NMLHandExo::setMotorLowerBound(uint8_t id, float lowerBound) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for lower bound update: " + String(id));
    return;
  }

  if (lowerBound > jointLimits_[index][1]) {
    debugPrint("Lower bound exceeds current upper bound for motor " + String(id));
    return;
  }

  jointLimits_[index][0] = lowerBound;
  debugPrint("Set lower bound for motor " + String(id) + " to " + String(lowerBound) + " deg");
}
void NMLHandExo::setMotorUpperBound(uint8_t id, float upperBound) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for upper bound update: " + String(id));
    return;
  }

  if (upperBound < jointLimits_[index][0]) {
    debugPrint("Upper bound below current lower bound for motor " + String(id));
    return;
  }

  jointLimits_[index][1] = upperBound;
  debugPrint("Set upper bound for motor " + String(id) + " to " + String(upperBound) + " deg");
}
String NMLHandExo::getMotorLimits(uint8_t id) {
  int index = getIndexById(id);
  if (index == -1) {
    return "[ERROR] Invalid motor ID: " + String(id);
  }

  float min = jointLimits_[index][0];
  float max = jointLimits_[index][1];
  return "[" + String(min, 2) + ", " + String(max, 2) + "]";
}
void NMLHandExo::setMotorLimits(uint8_t id, float lowerLimit, float upperLimit) {
  int index = getIndexById(id);
  if (index == -1) {
    debugPrint("Invalid motor ID for setting limits: " + String(id));
    return;
  }

  // Check if limits are valid
  if (lowerLimit >= upperLimit) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Invalid limits for motor %d: [%.2f, %.2f]", id, lowerLimit, upperLimit);
    debugPrint(buffer);
    return;
  }

  jointLimits_[index][0] = lowerLimit;
  jointLimits_[index][1] = upperLimit;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Set limits for motor %d: [%.2f, %.2f]", id, lowerLimit, upperLimit);
  debugPrint(buffer);

  // Update the control table items for the motor
  //dxl_.writeControlTableItem(UPPER_LIMIT, id, angleToTicks(upperLimit, index));
  //dxl_.writeControlTableItem(LOWER_LIMIT, id, angleToTicks(lowerLimit, index));
}


// ====================================================================================
// ============================ Torque commands =======================================
// ====================================================================================

bool NMLHandExo::getTorqueEnabledStatus(uint8_t id) {
  // Check if torque is enabled for the specified motor ID
  return dxl_.getTorqueEnableStat(id);
}

void NMLHandExo::enableTorque(uint8_t id, bool enable) {
  if (enable) {
    dxl_.torqueOn(id);
    debugPrint("Motor " + String(id) + " enabled");
  } else {
    dxl_.torqueOff(id);
    debugPrint("Motor " + String(id) + " disabled");
  }
}
void NMLHandExo::setCurrentLimit(uint8_t id, uint16_t current_mA) {
  int index = getIndexById(id);
  if (index == -1) {
      debugPrint(F("Invalid motor ID"));
      return;
  }
  currentLimits_[index] = current_mA;
  dxl_.writeControlTableItem(GOAL_CURRENT, id, current_mA);
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Set current limit for motor %d: %.2f mA", id, current_mA);
  debugPrint(buffer);
}
int16_t NMLHandExo::getCurrent(uint8_t id) {
  // Reads the current in mA from the motor's control table.
  return dxl_.readControlTableItem(PRESENT_CURRENT, id);
}
float NMLHandExo::getTorque(uint8_t id) {
  // Each unit = 2.69 mA; torque constant = 0.38 mN·m/mA = 0.00038 N·m/mA
  int16_t raw_current = NMLHandExo::getCurrent(id);
  float current_mA = raw_current * 2.69;
  float torque_Nm = current_mA * XL330_TORQUE_CONSTANT;
  return torque_Nm;  // in N·m
}
void NMLHandExo::setTorque(uint8_t id, float torque_Nm) {
  int index = getIndexById(id);
  if (index == -1) {
      debugPrint(F("Invalid motor ID"));
      return;
  }

  // Convert Nm to mA
  uint16_t current_mA = (uint16_t)(torque_Nm / XL330_TORQUE_CONSTANT);
  setCurrentLimit(id, current_mA);
  //debugPrint("Torque limit for motor " + String(id) + " set to " + String(torque_Nm, 4) + " Nm (current limit: " + String(current_mA) + " mA)");
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Torque limit for motor %d: set to %.2f N·m", id, torque_Nm);
  debugPrint(buffer);

}

// Velocity commands
void NMLHandExo::setVelocityLimit(uint8_t id, uint32_t vel) {
  dxl_.writeControlTableItem(PROFILE_VELOCITY, id, vel);
  debugPrint("Velocity limit set for motor " + String(id) + ": " + String(vel));
}
uint32_t NMLHandExo::getVelocityLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_VELOCITY, id);
}

// Acceleration commands
void NMLHandExo::setAccelerationLimit(uint8_t id, uint32_t acc) {
  dxl_.writeControlTableItem(PROFILE_ACCELERATION, id, acc);
  debugPrint("Acceleration limit set for motor " + String(id) + ": " + String(acc));
}
uint32_t NMLHandExo::getAccelerationLimit(uint8_t id) {
  return dxl_.readControlTableItem(PROFILE_ACCELERATION, id);
}

// Motor-specific commands
void NMLHandExo::rebootMotor(uint8_t id) {
  dxl_.reboot(id);
  debugPrint("Motor ID:" + String(id) + " rebooted");
}
void NMLHandExo::getMotorInfo(uint8_t id) {
  dxl_.ping(id);  // could be expanded to read Model Number, Version, etc.
  debugPrint("Pinged motor ID: " + String(id));
}
void NMLHandExo::setBaudRate(uint8_t id, uint32_t baudrate) {
  dxl_.writeControlTableItem(BAUD_RATE, id, baudrate);
  debugPrint("Motor ID:" + String(id) + " baudrate set to " + String(baudrate));
}
uint32_t NMLHandExo::getBaudRate(uint8_t id) {
  return dxl_.readControlTableItem(BAUD_RATE, id);
}
void NMLHandExo::setMotorLED(uint8_t id, bool state) {
  // Sets specified motor LED to the specified state
  if (state) {
    dxl_.ledOn(id);
  } else {
    dxl_.ledOff(id);
  }
}
void NMLHandExo::setAllMotorLED(bool state) {
  // Sets the state of all motor LEDs to the specified state
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    setMotorLED(id, state);
  }
}
void NMLHandExo::setMotorControlMode(uint8_t id, const String& mode){
  String m = mode;
  m.toUpperCase();

  if (m == "POSITION") {
    dxl_.setOperatingMode(id, OP_POSITION);
    debugPrint("Set motor " + String(id) + " to POSITION mode");
  } else if (m == "CURRENT_POSITION") {
    dxl_.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    debugPrint("Set motor " + String(id) + " to CURRENT_POSITION mode");
  } else if (m == "VELOCITY") {
    dxl_.setOperatingMode(id, OP_VELOCITY);
    debugPrint("Set motor " + String(id) + " to VELOCITY mode");
  } else {
    debugPrint("[ERROR] Unknown operating mode: " + m);
  }
}
void NMLHandExo::setMotorControlMode(const String& mode) {
  String m = mode;
  m.toUpperCase();
  for (int i = 0; i < numMotors_; i++) {
    uint8_t id = motorIds_[i];
    dxl_.torqueOff(id); // Turn off torque before changing mode
    NMLHandExo::setMotorControlMode(id, m); // Set the mode for each motor
  }
}
String NMLHandExo::getMotorControlMode() {
  String mode = "UNKNOWN";
  if (numMotors_ > 0) {
    mode = motorControlMode_; // return the internally tracked mode
  }
  return mode;
}

