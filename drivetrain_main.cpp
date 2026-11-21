#include "main.h"

#include <cstdint>
#include <cstdlib>

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
void initialize_autonomous_display();
void run_autonomous_route();

#ifdef RECF_ENCODER_TUNING
void reset_encoder_tuning();
bool update_encoder_tuning(pros::Controller& controller);
#endif

bool autonomous_movement_allowed() {
    if (pros::competition::is_disabled()) {
        return false;
    }
    return pros::competition::is_autonomous() ||
           (!pros::competition::is_connected() && master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1);
}

namespace {

pros::Motor claw_motor(7, pros::MotorGearset::green);
// The lift motors are mounted opposite each other.
pros::MotorGroup lift_motors({5, -6}, pros::MotorGearset::blue);

constexpr int claw_open_power = 40;
constexpr int claw_intake_power = -127;  // Full reverse power.
constexpr std::uint32_t claw_open_time_ms = 500;

void control_drive(pros::Controller& controller) {
    constexpr int joystick_deadband = 5;
    const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (throttle == PROS_ERR || turn == PROS_ERR) {
        chassis.drive_set(0, 0);
    } else if (std::abs(turn) > joystick_deadband) {
        const int power = static_cast<int>(chassis.opcontrol_curve_right(turn));
        chassis.drive_set(-power, power);
    } else {
        const int power = std::abs(throttle) <= joystick_deadband
            ? 0
            : static_cast<int>(chassis.opcontrol_curve_left(throttle));
        chassis.drive_set(power, power);
    }
}

class ClawControl {
public:
    void update(pros::Controller& controller) {
        const bool open_tapped = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1;
        const bool intake_held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) == 1;

        if (intake_held) {
            claw_motor.move(claw_intake_power);
            movement = Movement::intaking;
            return;
        }

        const std::uint32_t now = pros::millis();
        if (movement == Movement::intaking ||
            (movement == Movement::opening && now - opening_started_at >= claw_open_time_ms)) {
            claw_motor.move(0);
            movement = Movement::idle;
        }

        if (open_tapped && movement == Movement::idle) {
            claw_motor.move(claw_open_power);
            opening_started_at = now;
            movement = Movement::opening;
        }
    }

private:
    enum class Movement { idle, opening, intaking };

    Movement movement = Movement::idle;
    std::uint32_t opening_started_at = 0;
};

void control_lift(pros::Controller& controller) {
    constexpr int lift_speed_rpm = 95;
    const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (up == down) {
        lift_motors.brake();
    } else {
        lift_motors.move_velocity(up ? lift_speed_rpm : -lift_speed_rpm);
    }
}

void stop_motors() {
    chassis.drive_set(0, 0);
    claw_motor.move(0);
    lift_motors.brake();
}

}  // namespace

void initialize() {
    ez::ez_template_print();

    // Allow PROS devices to initialize.
    pros::delay(500);

    lift_motors.set_brake_mode_all(pros::MotorBrake::hold);

    // A curve value of 0.0 gives a linear joystick response.
    chassis.opcontrol_curve_default_set(0.0, 0.0);

    // Load the current starting PID values.
    // These can be tuned later.
    default_constants();

    // Initialize the driver controls and encoders without calibrating an IMU.
    chassis.odom_enable(false);
    chassis.opcontrol_curve_sd_initialize();
    chassis.drive_sensor_reset();
    initialize_autonomous_display();
}

void disabled() {
    stop_motors();
}

void competition_initialize() {}

void autonomous() {
    stop_motors();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    run_autonomous_route();
}

void opcontrol() {
    // Coast while driving so the robot rolls naturally when the sticks return
    // to center. Change to MOTOR_BRAKE_BRAKE or MOTOR_BRAKE_HOLD if preferred.
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    chassis.opcontrol_drive_sensors_reset();
    claw_motor.move(0);
    ClawControl claw;

#ifdef RECF_ENCODER_TUNING
    reset_encoder_tuning();
#endif

    // Build with -DRECF_TWO_CONTROLLERS to put the mechanisms on the partner controller.
#ifdef RECF_TWO_CONTROLLERS
    pros::Controller mechanism_controller(pros::E_CONTROLLER_PARTNER);
#else
    pros::Controller& mechanism_controller = master;
#endif

    // Require a fresh press after driver control begins.
    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    while (true) {
        const bool start_pressed = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == 1;
        if (start_pressed && !pros::competition::is_connected() && autonomous_movement_allowed()) {
            claw = ClawControl{};
#ifdef RECF_ENCODER_TUNING
            reset_encoder_tuning();
#endif
            autonomous();
            // Keep the drivetrain stopped until A is released, even if the route finished.
            while (!pros::competition::is_connected() && autonomous_movement_allowed()) {
                pros::delay(ez::util::DELAY_TIME);
            }
            chassis.drive_brake_set(MOTOR_BRAKE_COAST);
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
        }

        bool tuning_controls_drive = false;
#ifdef RECF_ENCODER_TUNING
        tuning_controls_drive = update_encoder_tuning(master);
#endif
        if (!tuning_controls_drive) {
            control_drive(master);
        }
        claw.update(mechanism_controller);
        control_lift(mechanism_controller);

        // Keep the EZ-Template loop timing.
        pros::delay(ez::util::DELAY_TIME);
    }
}
