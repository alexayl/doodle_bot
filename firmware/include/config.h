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
    WHEELBASE CALIBRATION:
    1. Command a 360° turn (or multiple full rotations for better accuracy)
    2. Measure the actual angle turned
    3. Calculate: new_wheelbase = current_wheelbase * (actual_angle / commanded_angle)
    
    SYMPTOMS:
    - Angle too wide (overturns)  ->  wheelbase value too narrow, increase it
    - Angle too narrow (underturns) ->  wheelbase value too wide, decrease it
    - Square extends beyond start ->  wheelbase too wide
    - Square doesn't close        ->  wheelbase too narrow
*/

#ifdef BOO
#define WHEEL_DIAMETER          (46.27f)
#define WHEELBASE               (151.0f)
#endif

#ifdef DOO
#define WHEEL_DIAMETER          (59.4f)
#define WHEELBASE               (183.0f)  // TODO: calibrate - see instructions above
#endif


#define STEPPER_CTRL_FREQ       (15.0f)    // control frequency (Hz) - higher = finer control
#define STEPPER_CTRL_PERIOD     (1.0f / STEPPER_CTRL_FREQ) // seconds per occurrence (100ms)

#define WHEEL_RADIUS            (WHEEL_DIAMETER / 2)
#define DOODLEBOT_RADIUS        (WHEELBASE / 2)   // distance from center of doodlebot to wheel
#define MAX_LINEAR_VELOCITY     (70.0f) // maximum linear velocity (mm/s)

#define MESSAGES_PER_QUEUE 200
#define ELEMENTS_PER_ARRAY 50
#define MOTION_PLAN_OUTPUT_SIZE 100
