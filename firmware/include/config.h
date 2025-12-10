#pragma once

// ----------------
// DEBUG STATEMENTS
// ----------------

// #define DEBUG_BLE
// #define DEBUG_INSTRUCTION_PARSER
// #define DEBUG_NAV
#define DEBUG_MOTION_EXECUTION
// #define DEBUG_INTERPOLATE
#define DEBUG_SERVO


// -------------
// CHOOSE DEVICE
// -------------

// #define BOO
#define DOO


// -------------------
// PHYSICAL PARAMETERS
// -------------------

/*
    - angle too wide        ->  wheelbase too narrow
    - angle too narrow      ->  wheelbase too wide
    - square extends beyond ->  wheelbase too wide
    - square not completed  ->  wheelbase too narrow
*/

#ifdef BOO
#define WHEEL_DIAMETER          (46.27f)
#define WHEELBASE               (151.0f)
#endif

#ifdef DOO
#define WHEEL_DIAMETER          (59.4f)
#define WHEELBASE               (176.0f) // too narrow
#endif


#define STEPPER_CTRL_FREQ       (5.0f)     // control frequency (Hz)
#define STEPPER_CTRL_PERIOD     (1.0f / STEPPER_CTRL_FREQ) // seconds per occurrence

#define WHEEL_RADIUS            (WHEEL_DIAMETER / 2)
#define DOODLEBOT_RADIUS        (WHEELBASE / 2)   // distance from center of doodlebot to wheel
#define MAX_LINEAR_VELOCITY     (70.0f) // maximum linear velocity (mm/s)

#define MESSAGES_PER_QUEUE 200
#define ELEMENTS_PER_ARRAY 50
