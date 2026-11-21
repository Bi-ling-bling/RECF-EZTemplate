#include "main.h"

// ------------------------------------------------------------
// EZ-TEMPLATE DRIVE PID STARTING VALUES
// These are the existing starting values from your project.
// You said tuning will be done later, so these are left as
// baseline values rather than treated as final tuning.
// ------------------------------------------------------------

void default_constants() {
    // Drive, heading, turn, and swing PID constants.
    chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
    chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
    chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
    chassis.pid_swing_constants_set(6.0, 0.0, 65.0);

    // Odometry PID values are retained for future use.
    chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);
    chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

    // PID exit conditions.
    chassis.pid_turn_exit_condition_set(
        90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms
    );
    chassis.pid_swing_exit_condition_set(
        90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms
    );
    chassis.pid_drive_exit_condition_set(
        90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms
    );
    chassis.pid_odom_turn_exit_condition_set(
        90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms
    );
    chassis.pid_odom_drive_exit_condition_set(
        90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms
    );

    // Motion chaining values.
    chassis.pid_turn_chain_constant_set(3_deg);
    chassis.pid_swing_chain_constant_set(5_deg);
    chassis.pid_drive_chain_constant_set(3_in);

    // Slew settings.
    chassis.slew_turn_constants_set(3_deg, 70);
    chassis.slew_drive_constants_set(3_in, 70);
    chassis.slew_swing_constants_set(3_in, 80);

    // Odometry settings retained for later, if needed.
    chassis.odom_turn_bias_set(0.9);
    chassis.odom_look_ahead_set(7_in);
    chassis.odom_boomerang_distance_set(16_in);
    chassis.odom_boomerang_dlead_set(0.625);

    chassis.pid_angle_behavior_set(ez::shortest);
}
