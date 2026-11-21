#include "main.h"

// ------------------------------------------------------------
// DRIVE TRAIN CONFIGURATION
// Four drive motors total: two on the left and two on the right.
//
// V5 Smart Ports:
//   Left side:  ports 1 and 2
//   Right side: ports 3 and 4
//
// A negative motor port reverses that motor in EZ-Template.
// The right side is reversed to match the original configuration.
// ------------------------------------------------------------

ez::Drive chassis(
    {1, 2},       // Left drive motors
    {-3, -4},     // Right drive motors
    0,            // No IMU fitted (0 is outside the V5 Smart Port range)
    3.25,         // Wheel diameter in inches (verify your actual wheel size)
    120           // Wheel RPM (verify cartridge and gearing)
);

// Defined in autons.cpp. This only sets the EZ-Template PID constants.
void default_constants();

namespace {

pros::Motor claw_motor(7, pros::MotorGearset::green);
pros::MotorGroup lift_motors({5, 6}, pros::MotorGearset::green);

void control_claw(pros::Controller& controller) {
    constexpr int claw_power = 40;  // Motor commands range from -127 to 127.
    const bool open = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    const bool close = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    if (open == close) {
        claw_motor.move(0);
    } else {
        claw_motor.move(open ? claw_power : -claw_power);
    }
}

void control_lift(pros::Controller& controller) {
    constexpr int lift_power = 60;
    const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (up == down) {
        lift_motors.brake();
    } else {
        lift_motors.move(up ? lift_power : -lift_power);
    }
}

}  // namespace

void initialize() {
    ez::ez_template_print();

    // Allow PROS devices to initialize.
    pros::delay(500);

    lift_motors.set_brake_mode_all(pros::MotorBrake::hold);

    // Tank-drive controller setup.
    // A curve value of 0.0 gives a linear joystick response.
    chassis.opcontrol_curve_buttons_toggle(false);
    chassis.opcontrol_curve_default_set(0.0, 0.0);
    chassis.opcontrol_drive_activebrake_set(0.0);

    // Load the current starting PID values.
    // These can be tuned later.
    default_constants();

    // Initialize the driver controls and encoders without calibrating an IMU.
    chassis.odom_enable(false);
    chassis.opcontrol_curve_sd_initialize();
    chassis.drive_sensor_reset();
}

void disabled() {}

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

    // Build with -DRECF_TWO_CONTROLLERS to put the mechanisms on the partner controller.
#ifdef RECF_TWO_CONTROLLERS
    pros::Controller mechanism_controller(pros::E_CONTROLLER_PARTNER);
#else
    pros::Controller& mechanism_controller = master;
#endif

    while (true) {
        // Tank drive:
        // Left joystick controls the left side.
        // Right joystick controls the right side.
        chassis.opcontrol_tank();
        control_claw(mechanism_controller);
        control_lift(mechanism_controller);

        // Keep the EZ-Template loop timing.
        pros::delay(ez::util::DELAY_TIME);
    }
}
