/**
 * @file gesture_library.h
 * @brief Header file for the gesture library used in the NML Hand Exoskeleton project.
 *
 */
#ifndef GESTURE_LIBRARY_H
#define GESTURE_LIBRARY_H
#include "config.h"


/// @brief Struct to represent a single gesture state.
struct GestureState {
    const char* name;              // e.g., "a", "open", "close"
    float jointAngles[N_MOTORS]; // joint angles for this state
};

/// @brief Struct to represent a gesture with its name and states.
struct GestureMap {
    const char* name;              // e.g., "pinch", "open"
    GestureState states[MAX_STATES_PER_GESTURE];
    int numStates;
};

/// @brief Number of gestures defined in the library.
extern GestureMap gestureLibrary[N_GESTURES];


/// @brief Helper function to find the index of a gesture by name.
inline int findGestureIndex(const String& gestureName) {
    for (int i = 0; i < N_GESTURES; i++) {
        if (gestureName.equalsIgnoreCase(gestureLibrary[i].name)) {
            return i;
        }
    }
    return -1;
}

/// @brief Helper function to find the index of a state in a gesture.
inline int findStateIndex(const GestureMap& gesture, const String& stateName) {
    for (int i = 0; i < gesture.numStates; i++) {
        if (stateName.equalsIgnoreCase(gesture.states[i].name)) {
            return i;
        }
    }
    return -1;
}

#endif
