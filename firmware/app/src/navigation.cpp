#include <zephyr/kernel.h>
#include <math.h>
#include "navigation.h"
#include "instruction_parser.h"

/* STATIC MEMBERS */
InstructionParser::GCodeCmd InstructionHandler::current_instruction_;
float MotionPlanner::theta_current = 0.0f;

k_msgq* MotionPlanner::nav_queue_ = nullptr;
k_msgq* MotionPlanner::step_queue_ = nullptr;

/* MOTION PLANNER */

int MotionPlanner::interpolate() {

    // Using linear interpolation
    int x_delta = current_instruction_.args[0].value;
    int y_delta = current_instruction_.args[1].value;

    // calculate r
    float r_delta = (float)sqrt(pow(x_delta, 2) + pow(y_delta, 2));

    // calculate theta
    float theta_desired = (float)atan2(y_delta, x_delta);

    // calculate change in delta
    float theta_delta = theta_desired - theta_current;

    // normalize theta (-180, 180)
    if (theta_delta < -180) {
        theta_delta += 360.0;
    } else if (theta_delta > 180) {
        theta_delta -= 360.0;
    }

    // put instr in queue
    NavCommand nav_command = {r_delta, theta_delta};
    k_msgq_put(nav_queue_, &nav_command, K_FOREVER);

    return 0;
}

int MotionPlanner::discretize() {

    // take navigation commands from nav_queue_
    NavCommand nav_command;
    k_msgq_get(nav_queue_, &nav_command, K_FOREVER);

    // turn them into wheel velocities
    float l_dist = -(nav_command.theta * 2 * PI * WHEEL_RADIUS) + nav_command.r;
    float r_dist = (nav_command.theta * 2 * PI * WHEEL_RADIUS) + nav_command.r;

    StepCommand step_command = {
        ((uint8_t)l_dist) * STEPPER_CTRL_FREQ,
        ((uint8_t)r_dist) * STEPPER_CTRL_FREQ
    };

    // push wheel velocities to step_queue_
    k_msgq_put(step_queue_, &step_command, K_FOREVER);

    return 0;
}

int MotionPlanner::consumeInstruction(const InstructionParser::GCodeCmd &current_instruction) {
    int ret;
    current_instruction_ = current_instruction;

    // turn gcode into motion commands
    ret = interpolate();
    if (ret < 0) {
        printk("ERROR: Interpolation not successful.\n");
    }

    // turn motion commands into wheel velocities
    while(k_msgq_peek(nav_queue_, NULL) != -ENOMSG) {
        ret = discretize();
        if (ret < 0) {
            printk("ERROR: Discretization not successful.\n");
        }
    }

    // wheel velocities are consumed by motor_control_handler()
    // start the timer that runs it if it is not currently running
    if (!k_timer_remaining_ticks(&motor_control_timer)) {
        k_timer_start(&motor_control_timer, K_FOREVER, K_SECONDS(STEPPER_CTRL_PERIOD));
    }
    
    return 0;
}


void MotionPlanner::motor_control_handler(k_timer *timer) {

    // get step command
    StepCommand step_command;
    if (!k_msgq_get(step_queue_, &step_command, K_NO_WAIT)) {
        k_timer_stop(&motor_control_timer);
        return;
    }

    // call stepper driver at specified velocity
    // TODO: call stepper driver

}


/* SERVO */

int ServoMover::consumeInstruction(const InstructionParser::GCodeCmd &gCodeCmd) {

    
    return 0;
}



/* THREAD */

void nav_thread(void *gcode_msgq_void, void *nav_cmd_msgq_void, void *step_cmd_msgq_void) {

    auto *gcode_msgq = (k_msgq *)(gcode_msgq_void);
    k_msgq *nav_cmd_msgq = (k_msgq *)(nav_cmd_msgq_void);
    k_msgq *step_cmd_msgq = (k_msgq *)(step_cmd_msgq_void);
    InstructionParser::GCodeCmd current_instruction;

    while(1) {
        // Block until message arrives
        k_msgq_get(gcode_msgq, &current_instruction, K_FOREVER);
        
        #ifdef DEBUG_NAV
        printk("Nav thread received %c%d command\n", current_instruction.code, current_instruction.number);
        #endif

        char code = current_instruction.code;
        int num = current_instruction.number;

        MotionPlanner motionPlanner(nav_cmd_msgq, step_cmd_msgq);
        ServoMover marker;
        ServoMover eraser;

        // command router
        if(code == 'G' && num == 1) {
            motionPlanner.consumeInstruction(current_instruction);

        } else if (code == 'M' && num == 280) {
            if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 0) {
                marker.consumeInstruction(current_instruction);
                
            } else if (current_instruction.args[0].letter == 'P' && current_instruction.args[0].value == 1) {
                eraser.consumeInstruction(current_instruction);

            } else {
                printk("Unhandled servo command: %c %d\n", current_instruction.args[0].letter, current_instruction.args[0].value);
            }

        } else {
            printk("Unhandled command: %c %d\n", code, num);
        }
        
    }

    return;
}
