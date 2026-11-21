#include "main.h"

// ------------------------------------------------------------
// DRIVE TRAIN CONFIGURATION
// Four drive motors total: two on the left and two on the right.
//
// Assumed ports based on your original main.cpp:
//   Left side:  ports 1 and 2
//   Right side: ports 4 and 5
//   IMU:        port 7
//
// A negative motor port reverses that motor in EZ-Template.
// The right side is reversed to match the original configuration.
// ------------------------------------------------------------

ez::Drive chassis(
    {1, 2},       // Left drive motors
    {-4, -5},     // Right drive motors
    7,            // IMU smart port
    3.25,         // Wheel diameter in inches (verify your actual wheel size)
    120           // Wheel RPM (verify cartridge and gearing)
);

// Defined in autons.cpp. This only sets the EZ-Template PID constants.
void default_constants();

void initialize() {
    ez::ez_template_print();

    // Allow PROS devices to initialize.
    pros::delay(500);

    // Tank-drive controller setup.
    // A curve value of 0.0 gives a linear joystick response.
    chassis.opcontrol_curve_buttons_toggle(false);
    chassis.opcontrol_curve_default_set(0.0, 0.0);
    chassis.opcontrol_drive_activebrake_set(0.0);

    // Load the current starting PID values.
    // These can be tuned later.
    default_constants();

    // Initialize the EZ-Template chassis and IMU.
    chassis.initialize();

    // Let the driver know whether the IMU calibrated.
    master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
    // No additional disabled behavior is needed for drivetrain-only code.
}

void competition_initialize() {
    // No autonomous selector is being used yet.
}

// Autonomous is intentionally left empty for now.
// We will add the actual autonomous route after the claw mechanism is ready.
void autonomous() {
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
}

void opcontrol() {
    // Coast while driving so the robot rolls naturally when the sticks return
    // to center. Change to MOTOR_BRAKE_BRAKE or MOTOR_BRAKE_HOLD if preferred.
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    while (true) {
        // Tank drive:
        // Left joystick controls the left side.
        // Right joystick controls the right side.
        chassis.opcontrol_tank();

        // Keep the EZ-Template loop timing.
        pros::delay(ez::util::DELAY_TIME);
    }
}
