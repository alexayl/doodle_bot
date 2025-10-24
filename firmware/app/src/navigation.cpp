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

    // normalize theta (-PI, PI)
    while (theta_delta > PI) {
        theta_delta -= 2 * PI;
    }
    while (theta_delta < -PI) {
        theta_delta += 2 * PI;
    }
    
    // Update current heading after turn
    theta_current = theta_desired;
    
    // Generate TWO commands: first turn in place, then move forward
    // Command 1: Turn in place (r=0, theta=theta_delta)
    if (fabs(theta_delta) > 0.01) {  // Only turn if angle is significant
        NavCommand turn_command = {0.0f, theta_delta};
        k_msgq_put(nav_queue_, &turn_command, K_FOREVER);
    }
    
    // Command 2: Move forward (r=r_delta, theta=0)
    if (r_delta > 0.01) {  // Only move if distance is significant
        NavCommand move_command = {r_delta, 0.0f};
        k_msgq_put(nav_queue_, &move_command, K_FOREVER);
    }

    return 0;
}

int MotionPlanner::discretize() {

    // take navigation commands from nav_queue_
    NavCommand nav_command;
    k_msgq_get(nav_queue_, &nav_command, K_FOREVER);

    #ifdef DEBUG_NAV
    printk("discretize: r=%.2f, theta=%.2f rad (%.1f deg)\n", 
           (double)nav_command.r, (double)nav_command.theta, 
           (double)(nav_command.theta * 180.0 / PI));
    #endif

    // Differential drive: arc length for each wheel
    // l_dist = distance left wheel travels = R_left * theta
    // r_dist = distance right wheel travels = R_right * theta
    // Where R_left/right are the radii from the ICC (instantaneous center of curvature)
    
    // For a turn by angle theta_delta and forward movement r:
    // Left wheel:  arc = r - (theta_delta * DOODLEBOT_RADIUS)
    // Right wheel: arc = r + (theta_delta * DOODLEBOT_RADIUS)
    
    float l_dist = nav_command.r - (nav_command.theta * DOODLEBOT_RADIUS);
    float r_dist = nav_command.r + (nav_command.theta * DOODLEBOT_RADIUS);
    
    #ifdef DEBUG_NAV
    printk("  l_dist=%.2f, r_dist=%.2f\n", (double)l_dist, (double)r_dist);
    #endif
    
    // Convert to velocities
    int left_vel = (int)(l_dist * STEPPER_CTRL_FREQ);
    int right_vel = (int)(r_dist * STEPPER_CTRL_FREQ);
    
    #ifdef DEBUG_NAV
    printk("  before clamp: left_vel=%d, right_vel=%d\n", left_vel, right_vel);
    #endif
    
    // Clamp to int16_t range [-32768, 32767]
    // In practice, we'll probably want a smaller range for safety
    left_vel = (left_vel < -32768) ? -32768 : ((left_vel > 32767) ? 32767 : left_vel);
    right_vel = (right_vel < -32768) ? -32768 : ((right_vel > 32767) ? 32767 : right_vel);
    
    StepCommand step_command = {
        (int16_t)left_vel,
        (int16_t)right_vel
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
    // Process all nav commands in the queue
    while(k_msgq_num_used_get(nav_queue_) > 0) {
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
    printk("Stepper command: left_velocity=%d, right_velocity=%d\n", step_command.left_velocity, step_command.right_velocity);
    

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

    // Create handler objects once, before the loop
    MotionPlanner motionPlanner(nav_cmd_msgq, step_cmd_msgq);
    ServoMover marker;
    ServoMover eraser;

    while(1) {
        // Block until message arrives
        k_msgq_get(gcode_msgq, &current_instruction, K_FOREVER);
        
        #ifdef DEBUG_NAV
        printk("Nav thread received %c%d command\n", current_instruction.code, current_instruction.number);
        #endif

        char code = current_instruction.code;
        int num = current_instruction.number;

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
