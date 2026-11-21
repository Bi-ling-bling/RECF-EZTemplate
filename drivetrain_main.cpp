#include "main.h"

#include <cmath>
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
void initialize_autonomous_selector();
void run_autonomous_route();

#ifdef RECF_ENCODER_TUNING
void reset_encoder_tuning();
bool update_encoder_tuning(pros::Controller& controller);
#endif

namespace {

pros::Motor claw_motor(7, pros::MotorGearset::green);
pros::MotorGroup lift_motors({5, 6}, pros::MotorGearset::green);

constexpr int claw_power = 40;  // Motor commands range from -127 to 127.
constexpr std::uint32_t claw_open_time_ms = 500;
constexpr std::uint32_t claw_close_wait_ms = 250;
constexpr std::uint32_t claw_intake_time_ms = 500;
constexpr int lift_auton_speed_rpm = 45;
constexpr double lift_position_tolerance_degrees = 8.0;
constexpr std::uint32_t lift_timeout_ms = 3000;

void control_drive(pros::Controller& controller) {
    constexpr int joystick_deadband = 5;
    const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (throttle == PROS_ERR || turn == PROS_ERR) {
        chassis.drive_set(0, 0);
    } else if (turn < -joystick_deadband) {
        // Back up only the inside side to pivot around the other side.
        chassis.drive_set(static_cast<int>(chassis.opcontrol_curve_right(turn)), 0);
    } else if (turn > joystick_deadband) {
        chassis.drive_set(0, static_cast<int>(chassis.opcontrol_curve_right(-turn)));
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
        const bool open_tapped = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) == 1;
        const bool intake_held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1;

        if (intake_held) {
            claw_motor.move(-claw_power);
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
            claw_motor.move(claw_power);
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
    const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    if (up == down) {
        lift_motors.brake();
    } else {
        lift_motors.move_velocity(up ? lift_speed_rpm : -lift_speed_rpm);
    }
}

bool run_claw_for(int power, std::uint32_t duration_ms) {
    if (pros::competition::is_disabled() || claw_motor.move(power) == PROS_ERR) {
        return false;
    }
    const auto started_at = pros::millis();
    while (!pros::competition::is_disabled() && pros::millis() - started_at < duration_ms) {
        pros::delay(ez::util::DELAY_TIME);
    }
    claw_motor.move(0);
    return !pros::competition::is_disabled();
}

bool move_lift_to(double target_degrees) {
    if (pros::competition::is_disabled() ||
        lift_motors.move_absolute(target_degrees, lift_auton_speed_rpm) == PROS_ERR) {
        lift_motors.brake();
        return false;
    }

    const auto started_at = pros::millis();
    while (!pros::competition::is_disabled() && pros::millis() - started_at < lift_timeout_ms) {
        const double left = lift_motors.get_position(0);
        const double right = lift_motors.get_position(1);
        if (!std::isfinite(left) || !std::isfinite(right)) {
            break;
        }
        if (std::abs(target_degrees - left) <= lift_position_tolerance_degrees &&
            std::abs(target_degrees - right) <= lift_position_tolerance_degrees) {
            lift_motors.brake();
            return true;
        }
        pros::delay(ez::util::DELAY_TIME);
    }
    lift_motors.brake();
    return false;
}

}  // namespace

bool prepare_autonomous_mechanisms() {
    return claw_motor.move(0) != PROS_ERR &&
           lift_motors.set_encoder_units_all(pros::MotorUnits::degrees) != PROS_ERR &&
           lift_motors.tare_position_all() != PROS_ERR;
}

bool autonomous_grab() {
    // Give the claw time to close mechanically with the motor stopped before intaking.
    return run_claw_for(claw_power, claw_open_time_ms) &&
           run_claw_for(0, claw_close_wait_ms) &&
           run_claw_for(-claw_power, claw_intake_time_ms);
}

bool autonomous_score(double lift_degrees) {
    return move_lift_to(lift_degrees) && run_claw_for(claw_power, claw_open_time_ms);
}

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
    initialize_autonomous_selector();
}

void disabled() {
    chassis.drive_set(0, 0);
    claw_motor.move(0);
    lift_motors.brake();
}

void competition_initialize() {}

void autonomous() {
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

    while (true) {
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
