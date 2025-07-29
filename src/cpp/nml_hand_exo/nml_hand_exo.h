/**
 * @file nml_hand_exo.h
 * @brief API header for the NML Hand Exoskeleton device.
 *
 * This file declares the NMLHandExo class and helper utilities for managing
 * and controlling the exoskeleton using Dynamixel servos.
 */
#ifndef NML_HAND_EXO_H
#define NML_HAND_EXO_H

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

/// @brief Verbose output toggle for debugging.
extern bool VERBOSE;

/// @brief Bluetooth serial stream used for commands.
extern Stream& COMMAND_SERIAL;

extern Stream* debugStream;

/// @brief Debugging print helper function.
/// @param msg The message to print.
void debugPrint(const String& msg);

/// @brief Mode press function
void onModeButtonPress();

/// @brief Enum to manage the operating mode of the exo
enum ExoOperatingMode {
  FREE = 0,
  GESTURE_FIXED,
  GESTURE_CONTINUOUS,
  GESTURE_CALIBRATION
};


/// @brief Class to manage the NML Hand Exoskeleton, providing initialization, motor control, and telemetry.
class NMLHandExo {
  public:
    /// @brief Constructor.
    /// @param ids Pointer to array of motor IDs.
    /// @param numMotors Number of motors in the device.
    /// @param jointLimits Pointer to 2D array defining [min, max] joint limits in degrees.
    /// @param homeState Optional array of home positions (in degrees). Defaults to zero offsets.
    NMLHandExo(const uint8_t* ids, uint8_t numMotors, const float jointLimits[][2], const float* homeState = nullptr);

    /// @brief Destructor.
    ~NMLHandExo() {
      delete[] jointLimits_;
      delete[] zeroOffsets_;
      delete[] currentLimits_;
    }

    // -----------------------------------------------------------
    // Utility functions
    // -----------------------------------------------------------

    /// @brief Initialize the serial port for Dynamixel communication.
    /// @param baud Baud rate to initialize.
    void initializeSerial(int baud);

    /// @brief Initialize all motors: disables torque, sets position mode, then re-enables torque.
    void initializeMotors();

    /// @brief Gets the motor ID according to the passed index
    /// @param index Index of array
    uint8_t getMotorIDByIndex(const int index);

    /// @brief Get the current operating mode of the motors
    /// @return The motor control mode.
    String getMotorControlMode();

    /// @brief Set the operating mode of the motors
    /// @param name Name of the mode (e.g. "position", "current_position", "velocity").
    void setMotorControlMode(const String& name);

    /// @brief Get the motor ID from a user-supplied token (either name or ID as a string).
    /// @param token The token string (e.g. "WRIST" or "1").
    /// @return The motor ID or -1 if not found.
    int getMotorID(const String& token);

    /// @brief Get the index of a motor in the internal arrays from its ID.
    /// @param id The motor ID.
    /// @return The index in the motor array or -1 if not found.
    int getIndexById(uint8_t id);

    /// @brief Get the motor ID by name.
    /// @param name Name of the motor (e.g. "WRIST").
    /// @return The motor ID or -1 if not found.
    int getMotorIDByName(const String& name);

    // @brief Get the name of the motor by index
    // @param index Index of array
    //const char* getMotorName(int index);

    /// @brief Set the unique names for motors (Must match the number of IDs)
    /// @param names A list of "names" separated by comma.
    void setMotorNames(const char* const* names);

    /// @brief Get the motor name from its ID.
    /// @param id The motor ID.
    /// @return The name of the motor.
    String getMotorNameByID(uint8_t id);

    /// @brief Convert a relative angle (degrees) to Dynamixel tick counts.
    /// @param angle_deg Angle in degrees.
    /// @param index Index of the motor.
    /// @return Ticks equivalent to the angle.
    int angleToTicks(float angle_deg, int index);

    /// @brief Calibrate the zero offset for a motor by reading its current position.
    /// @param id Motor ID.
    void setZeroOffset(uint8_t id);

    /// @brief Get the stored zero offset for a motor.
    /// @param id Motor ID.
    /// @return Zero offset in degrees.
    float getZeroOffset(uint8_t id);

    /// @brief Reset zero offsets for all motors using their current positions.
    void resetAllZeros();

    /// @brief Get a string summarizing the device information.
    // @return Information string.
    String getDeviceInfo();

    /// @brief Get the number of motors
    /// @return integer, number of motors.
    int getMotorCount();

    // -----------------------------------------------------------
    // Calibration functions
    // -----------------------------------------------------------

    /// @brief Start the calibration process for the exoskeleton.
    /// @param enableTimedCalibration If true, enables timed calibration mode.
    /// @param duration Duration in seconds for timed calibration.
    void beginCalibration(bool enableTimedCalibration, int duration);

    /// @brief Update the calibration state, checking if the calibration is complete.
    void updateCalibration();

    /// @brief Check if the exoskeleton is currently calibrating.
    /// @return True if in calibration mode, false otherwise.
    bool isExoCalibrating();

    // -----------------------------------------------------------
    // Mode functions
    // -----------------------------------------------------------

    /// @brief Assign pin for mode switch interrupt.
    /// @param pin Interrupt pin.
    void setModeSwitchButton(int pin);

    /// @brief Define the operating mode for exo.
    /// @param name Operating mode ("GESTURE_FIXED", "GESTURE_CONTINUOUS", "FREE")
    void setExoOperatingMode(const String& name);

    /// @brief Get the current operating mode for exo.
    /// @return The current operating mode as a string.
    String getExoOperatingMode();

    /// @brief Get the current operating mode as an enum.
    /// @return The current operating mode as an ExoOperatingMode enum.
    ExoOperatingMode getExoOperatingModeEnum();

    /// @brief Check if the mode switch button was pressed.
    bool checkModeSwitchButtonPressed();

    /// @brief Update the exo state, including checking for button pressed, mode switching, and internal routines
    void update();

    /// @brief Cycle through the exo operating modes.
    void cycleExoOperatingMode();


    // -----------------------------------------------------------
    // Position commands
    // -----------------------------------------------------------

    /// @brief Get the relative angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Relative angle in degrees.
    float getRelativeAngle(uint8_t id);

    /// @brief Command the motor to a relative angle.
    /// @param id Motor ID.
    /// @param angleDeg Relative angle in degrees.
    void setRelativeAngle(uint8_t id, float angleDeg);

    /// @brief Get the absolute angle (degrees) of a motor.
    /// @param id Motor ID.
    /// @return Absolute angle in degrees.
    float getAbsoluteAngle(uint8_t id);

    /// @brief Command the motor to an absolute angle.
    /// @param id Motor ID.
    /// @param angleDeg Absolute angle in degrees.
    void setAbsoluteAngle(uint8_t id, float angleDeg);

    /// @brief Get the stored zero angle of a motor.
    /// @param id Motor ID.
    /// @return Zero angle in degrees.
    float getZeroAngle(uint8_t id);

    /// @brief Home the motor to its stored zero position.
    /// @param id Motor ID.
    void setHome(uint8_t id);

    /// @brief Home all motors to their stored zero positions.
    void homeAllMotors();

    /// @brief Command the motor to an angle by ID.
    /// @param id Motor ID.
    /// @param angleDeg Angle in degrees.
    void setAngleById(uint8_t id, float angleDeg);

    /// @brief Command a motor using an alias (name) and angle.
    /// @param alias Motor name (e.g. "WRIST").
    /// @param angleDeg Angle in degrees.
    void setAngleByAlias(const String& alias, float angleDeg);

    /// @brief Set the lower joint limit (in degrees) for a motor.
    /// @param id Motor ID.
    /// @param lowerBound New lower bound in degrees.
    void setMotorLowerBound(uint8_t id, float lowerBound);

    /// @brief Set the upper joint limit (in degrees) for a motor.
    /// @param id Motor ID.
    /// @param upperBound New upper bound in degrees.
    void setMotorUpperBound(uint8_t id, float upperBound);

    /// @brief Get the joint angle limits for a motor.
    /// @param id Motor ID.
    /// @return A string in the format "[min, max]" or error message.
    String getMotorLimits(uint8_t id);

    /// @brief Set the joint angle limits for a motor.
    /// @param id Motor ID.
    /// @param lowerLimit New lower limit in degrees.
    /// @param upperLimit New upper limit in degrees.
    void setMotorLimits(uint8_t id, float lowerLimit, float upperLimit);



    // -----------------------------------------------------------
    // Torque commands
    // -----------------------------------------------------------

    /// @brief Check if torque is enabled for a motor.
    /// @param id Motor ID.
    bool getTorqueEnabledStatus(uint8_t id);

    /// @brief Enable or disable torque for a motor.
    /// @param id Motor ID.
    /// @param enable True to enable torque, false to disable.
    void enableTorque(uint8_t id, bool enable);

    /// @brief Get the current draw from a motor.
    /// @param id Motor ID.
    /// @return Raw current value.
    int16_t getCurrent(uint8_t id);

    /// @brief Set the current limit for a motor.
    /// @param id Motor ID.
    /// @param current_mA Current limit in milliamps.
    void setCurrentLimit(uint8_t id, uint16_t current_mA);

    /// @brief Set the calculated torque in N·m for a motor.
    /// @param id Motor ID.
    /// @return Torque in Newton-meters.
    void setTorque(uint8_t id, float torque_Nm);

    /// @brief Get the calculated torque in N·m for a motor.
    /// @param id Motor ID.
    /// @return Torque in Newton-meters.
    float getTorque(uint8_t id);

    // -----------------------------------------------------------
    // Velocity commands
    // -----------------------------------------------------------

    /// @brief Set the velocity limit of a motor.
    /// @param id Motor ID.
    /// @param vel Velocity limit.
    void setVelocityLimit(uint8_t id, uint32_t vel);

    /// @brief Get the velocity limit of a motor.
    /// @param id Motor ID.
    /// @return Velocity limit.
    uint32_t getVelocityLimit(uint8_t id);

    // -----------------------------------------------------------
    // Acceleration commands
    // -----------------------------------------------------------

    /// @brief Set the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @param acc Acceleration limit.
    void setAccelerationLimit(uint8_t id, uint32_t acc);

    /// @brief Get the acceleration limit of a motor.
    /// @param id Motor ID.
    /// @return Acceleration limit.
    uint32_t getAccelerationLimit(uint8_t id);

    // -----------------------------------------------------------
    // Motor-specific commands
    // -----------------------------------------------------------

    /// @brief Reboot a motor.
    /// @param id Motor ID.
    void rebootMotor(uint8_t id);

    /// @brief Ping a motor to verify communication.
    /// @param id Motor ID.
    void getMotorInfo(uint8_t id);

    /// @brief Set the baud rate of a motor.
    /// @param id Motor ID.
    /// @param baudrate New baud rate.
    void setBaudRate(uint8_t id, uint32_t baudrate);

    /// @brief Get the baud rate of a motor.
    /// @param id Motor ID.
    /// @return Baud rate.
    uint32_t getBaudRate(uint8_t id);

    /// @brief Set the LED state of a motor.
    /// @param id Motor ID.
    /// @param state True for on, false for off.
    void setMotorLED(uint8_t id, bool state);

    /// @brief Set the LED state of all motors.
    /// @param state True for on, false for off.
    void setAllMotorLED(bool state);

    /// @brief Set the current control mode of a motor.
    /// @param id Motor ID.
    /// @param mode Control mode as a string (e.g. "POSITION", "CURRENT_POSITION", "VELOCITY").
    void setMotorControlMode(uint8_t id, const String& mode);

    /// @brief Get the current control mode of a motor.
    /// @param id Motor ID.
    /// @return Control mode as a string.
    String getMotorControlMode(uint8_t id);

    /// @brief Set the control mode for all motors.
    /// @param mode Control mode as a string (e.g. "POSITION", "CURRENT_POSITION", "VELOCITY").
    void setMotorMode(const String& mode);

    /// @brief Get the current control mode of all motors.
    /// @return Control mode as a string.
    String getMotorMode();

    /// @brief Current software version.
    static constexpr const char* VERSION = "0.2.10";

  private:
    /// @brief Dynamixel2Arduino object for motor communication.
    Dynamixel2Arduino dxl_;              // Handle to Dynamixel object

    /// @brief Pointer to array of motor IDs.
    const uint8_t* motorIds_;                 // List of motor IDs passed by the user

    /// @brief Pointer to array of motor names.
    const char* const* motorNames_;

    /// @brief Number of motors configured.
    uint8_t numMotors_ = 0;                  // Number of motors being used, should be detected by the length of motor ids passed

    /// @brief Pointer to 2D array of joint limits [min, max] for each motor.
    float (*jointLimits_)[2];      // Pointer to 2D array of joint limits

    /// @brief Array of zero offsets for each motor.
    float* zeroOffsets_;                 // Track absolute zero positions

    /// @brief Array of current limits for each motor
    uint16_t* currentLimits_;

    /// @brief Mode switch pin
    int modeSwitchPin = -1;

    /// @brief Mode switch flag for interrupt callback
    static volatile bool modeSwitchFlag;

    /// @brief Last interrupt time for mode switch button
    bool lastButtonState = false; // Last state of the mode switch button

    /// @brief Current state of the mode switch button
    int buttonState = HIGH;

    /// @brief Last debounce time for mode switch button
    unsigned long lastDebounceTime = 0;

    /// @brief Debounce delay for mode switch button
    const unsigned long debounceDelay = 50;  // 50 ms debounce

    /// @brief Operating mode of motors.
    String motorControlMode_;            

    /// @brief Operating mode of exo
    ExoOperatingMode exoMode_ = FREE; // Default mode is free

    /// brief Flag to indicate if the exoskeleton is currently calibrating.
    bool isCalibrating = false;

    // @brief Flag for calibration timed mode
    bool calibrationTimedMode = false;

    /// @brief Start time for calibration.
    unsigned long calibrationStartTime;

    /// @brief Duration for calibration in milliseconds.
    unsigned long calibrationDuration;

};

#endif // NML_HAND_EXO_H