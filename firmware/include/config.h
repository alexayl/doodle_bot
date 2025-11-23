#pragma once

// #define DEBUG_BLE
// #define DEBUG_INSTRUCTION_PARSER
// #define DEBUG_NAV
#define DEBUG_MOTION
#define DEBUG_INTERPOLATE

#define WHEEL_DIAMETER          (47.30)
#define WHEELBASE               (154.0)

#define STEPPER_CTRL_FREQ       (5.0f)     // control frequency (Hz)

#define WHEEL_RADIUS            (WHEEL_DIAMETER / 2)
#define DOODLEBOT_RADIUS        (WHEELBASE / 2)   // distance from center of doodlebot to wheel
#define MAX_LINEAR_VELOCITY     (80.0f) // maximum linear velocity (mm/s)
