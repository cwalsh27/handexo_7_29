#include "gesture_library.h"

GestureMap gestureLibrary[N_GESTURES] = {
    // This is where you can create your own gestures and their states
    // Each gesture is a map of states, where each state has a vector of joint angles
    {"grasp", {{"open", {200.0, 126.0, 242.0, 195.0, 159.0, 214.0}},
               {"close", {200.0, 147.0, 302.0, 142.0, 226.0, 138.0}}
              }, 2},
    {"point", {{"a", {260.0, 147.0, 242.0, 142.0, 226.0, 138.0}}
              }, 1},

    // You can add multiple states to a gesture, especially if it is being used for manipulation or grasping
    {"pinch", {{"open", {260.0, 126.0, 242.0, 142.0, 226.0, 138.0}}, 
               {"close", {260.0, 150.0, 302.0, 142.0, 226.0, 138.0}}
              }, 2}
};