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
    
    Use the rotation test instead of a square - it isolates wheelbase errors
    from wheel diameter errors, making calibration much easier.
    
    METHOD:
    1. Mark the robot's starting heading on the ground
    2. Send the calibration file: tests/app/ble_gcode_sender/wheelbase_calibration.gcode
       (This performs 4 full rotations = 1440° total)
    3. Measure the heading error after the robot stops:
       - POSITIVE if robot overturned (past the mark)
       - NEGATIVE if robot underturned (before the mark)
    4. Calculate the corrected wheelbase:
       new_wheelbase = current_wheelbase × (1440 / (1440 + error))
    
    Or use the Python script: cv_software/app/calibrate_wheelbase.py
    
    SYMPTOMS:
    - Robot overturns  → configured WHEELBASE too large, DECREASE it
    - Robot underturns → configured WHEELBASE too small, INCREASE it
    
    WHY ROTATION TEST IS BETTER THAN SQUARE:
    - Square test mixes wheel diameter and wheelbase errors
    - Rotation test ONLY depends on wheelbase (isolates the variable)
    - Multiple rotations amplify small errors into measurable angles
      (1% error × 4 rotations = 14.4° visible error)
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
#define MAX_LINEAR_VELOCITY     (50.0f) // maximum linear velocity (mm/s)

#define MESSAGES_PER_QUEUE 200
#define ELEMENTS_PER_ARRAY 50
#define MOTION_PLAN_OUTPUT_SIZE 150
