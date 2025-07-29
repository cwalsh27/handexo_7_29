#include "gesture_library.h"
#include "gesture_controller.h"

GestureController::GestureController(NMLHandExo& exo)
  : exo_(exo),
    cycleGesturePin(-1),
    gestureStateSwitchPin(-1),
    lastCycleGestureDebounceTime(0),
    lastGestureStateDebounceTime(0),
    lastCycleGestureButtonState(HIGH),
    lastGestureStateButtonState(HIGH),
    cycleGestureButtonState(HIGH),
    gestureStateButtonState(HIGH) 
{
    currentGesture_ = gestureLibrary[0].name;
    currentGestureState_ = gestureLibrary[0].states[0].name;
    numGestures_ = N_GESTURES;
}

void GestureController::executeGesture(const String& gesture, const String& state) {
  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return;
  }

  int sIdx = findStateIndex(gestureLibrary[gIdx], state);
  if (sIdx == -1) {
    debugPrint("[GestureController] Unknown state: " + state + " for gesture " + gesture);
    return;
  }

  float* angles = gestureLibrary[gIdx].states[sIdx].jointAngles;
  for (int i = 0; i < exo_.getMotorCount(); i++) {
    uint8_t id = exo_.getMotorIDByIndex(i);  // ID from index
    exo_.setAbsoluteAngle(id, angles[i]);
    debugPrint("Setting motor " + String(id) + " to angle " + String(angles[i], 2) + " deg");
  }

  currentGesture_ = gesture;
  currentGestureState_ = state;
  debugPrint("[GestureController] Executed gesture: " + gesture + ", state: " + state);
}

void GestureController::executeCurrentGestureNewState(const String& state) {
  // Get the current gesture
  String gesture = getCurrentGesture();

  // Find the gesture index
  int gIdx = findGestureIndex(gesture);
  if (gIdx == -1) {
    debugPrint("[GestureController] Unknown gesture: " + gesture);
    return;
  }

  // Check if the state exists for the gesture
  int sIdx = findStateIndex(gestureLibrary[gIdx], state);
  if (sIdx == -1) {
    debugPrint("[GestureController] Error: State '" + state + "' not found for gesture '" + gesture + "'.");
    return;
  }

  // Execute the gesture with the specified state
  executeGesture(gesture, state);
}

void GestureController::setCycleGestureButton(const int pin) {
  cycleGesturePin = pin;
  pinMode(pin, INPUT_PULLUP);
  delay(100);  // Give pin state time to settle

  lastCycleGestureButtonState = HIGH;
  cycleGestureButtonState = HIGH;
  lastCycleGestureDebounceTime = 0;
  debugPrint("Gesture state switch button set on pin " + String(cycleGesturePin));
}

void GestureController::setGestureStateSwitchButton(const int pin) {
  gestureStateSwitchPin = pin;
  pinMode(pin, INPUT_PULLUP);
  delay(100);  // Give pin state time to settle

  lastGestureStateButtonState = HIGH;
  gestureStateButtonState = HIGH;
  lastGestureStateDebounceTime = 0;
  debugPrint("Gesture state switch button set on pin " + String(gestureStateSwitchPin));
}

bool GestureController::checkGestureStateButtonPressed() {
  if (gestureStateSwitchPin == -1) return false;
  int reading = digitalRead(gestureStateSwitchPin);
  if (reading != lastGestureStateButtonState) {
    lastGestureStateDebounceTime = millis();
  }

  if ((millis() - lastGestureStateDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
    if (reading != gestureStateButtonState) {
      gestureStateButtonState = reading;
      if (gestureStateButtonState == LOW) {
        // === Button was pressed ===
        return true;
      }
    }
  }
  lastGestureStateButtonState = reading;
  return false;
}

bool GestureController::checkCycleGestureButtonPressed() {
    if (cycleGesturePin == -1) return false;
    int reading = digitalRead(cycleGesturePin);
    if (reading != lastCycleGestureButtonState) {
        lastCycleGestureDebounceTime = millis();
    }

    if ((millis() - lastCycleGestureDebounceTime) > BUTTON_DEBOUNCE_DURATION) {
      if (reading != cycleGestureButtonState) {
          cycleGestureButtonState = reading;
          if (cycleGestureButtonState == LOW) {
            // === Button was pressed ===
            return true;
          }
        }
    }
    lastCycleGestureButtonState = reading;
    return false;
}

void GestureController::cycleGesture() {
    // Cycle through the gestures
    debugPrint("Current gesture: " + currentGesture_);
    int gIdx = findGestureIndex(currentGesture_);
    if (gIdx == -1) {
        debugPrint("[GestureController] Error: current gesture not found.");
        return;
    }
    int newIdx = (gIdx + 1) % numGestures_;  // Cycle through gestures
    debugPrint("New gesture index: " + String(newIdx));
    if (newIdx == 0) {
        debugPrint("[GestureController] Wrapped back to first gesture.");
    }

    currentGesture_ = gestureLibrary[newIdx].name;

    if (gestureLibrary[newIdx].numStates > 0) {
      currentGestureState_ = gestureLibrary[newIdx].states[0].name;
      debugPrint("[GestureController] Cycling gesture to: '" + currentGesture_ +"' (index: " + String(newIdx) + "), state: '" + currentGestureState_ + "'");
      executeGesture(currentGesture_, currentGestureState_);
    } else {
      debugPrint("[GestureController] Gesture " + currentGesture_ + " has no states.");
    }
}

String GestureController::getCurrentGesture() {
    return currentGesture_;
}

String GestureController::getCurrentGestureState() {
    return currentGestureState_;
}

void GestureController::cycleGestureState() {
    // Cycle through the states of the current gesture
    int gIdx = findGestureIndex(currentGesture_);
    if (gIdx == -1) {
        debugPrint("[GestureController] Error: current gesture not found.");
        return;
    }    
    debugPrint("Gesture index: " + String(gIdx));
    int currentStateIdx = findStateIndex(gestureLibrary[gIdx], currentGestureState_);
    if (currentStateIdx == -1) {
        debugPrint("[GestureController] Error: state not found for gesture: " + currentGesture_);
        return;
    }

    int nextStateIdx = (currentStateIdx + 1) % gestureLibrary[gIdx].numStates;
    String newState = gestureLibrary[gIdx].states[nextStateIdx].name;

    debugPrint("[GestureController] Cycling state to: " + newState +
               " (index: " + String(nextStateIdx) + ")");
    executeGesture(currentGesture_, newState);
}

void GestureController::update() {
    // Check if the gesture state button was pressed
    if (checkCycleGestureButtonPressed()) {
        debugPrint("[GestureController] cycle gesture button pressed");
        String exo_mode = exo_.getExoOperatingMode();
        if (exo_mode == "GESTURE_FIXED" || exo_mode == "GESTURE_CONTINUOUS") {
            // toggle the gesture index, call executeGesture(...)
            cycleGesture();
        }
    }

    // Check if the gesture state button was pressed
    if (checkGestureStateButtonPressed()) {
        debugPrint("[GestureController] gesture state button pressed");
        String gesture = getCurrentGesture();
        if (exo_.getExoOperatingMode() == "GESTURE_FIXED" || exo_.getExoOperatingMode() == "GESTURE_CONTINUOUS") {
            // toggle the gesture state, call executeGesture(...)
            cycleGestureState();
        }
    }
}


