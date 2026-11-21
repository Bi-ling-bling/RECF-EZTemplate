#include "main.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <numbers>
#include <optional>
#include <vector>

// Mechanism actions use the same motor objects as driver control in main.cpp.
bool prepare_autonomous_mechanisms();
bool autonomous_grab();
bool autonomous_score(double lift_degrees);
bool autonomous_movement_allowed();

namespace {

constexpr double distance_scale = 1.0;  // Multiply by measured distance / reported distance.
constexpr double track_width_inches = 12.0;  // Calibrate the effective wheel-center spacing.
constexpr double backward_inches = 16.077569;
constexpr double forward_inches = 24.69417;
constexpr double lift_raise_motor_degrees = 90.0;  // Scoring height from the lowered start.
constexpr double position_tolerance_inches = 0.75;
constexpr std::uint32_t settle_time_ms = 150;
constexpr std::uint32_t timeout_ms = 4000;

struct DriveSettings {
    double kP;
    double kD;
    int max_power;
};

constexpr DriveSettings straight_settings{8.0, 16.0, 45};
constexpr DriveSettings turn_settings{8.0, 16.0, 35};

struct WheelTravel {
    double left;
    double right;
};

struct EncoderMove {
    ez::PID left_pid;
    ez::PID right_pid;
    WheelTravel start{};
    double ticks_per_inch = 0.0;
    int max_power = 0;
    std::uint32_t started_at = 0;
    std::optional<std::uint32_t> settled_at;
};

enum class MoveResult { running, settled, failed };
enum class Step { backward, forward, left, right, score, grab };

constexpr Step route[] = {
    Step::backward, Step::left, Step::score,
    Step::backward, Step::right, Step::forward, Step::grab,
    Step::backward, Step::left, Step::forward, Step::score,
    Step::backward, Step::right, Step::forward, Step::grab,
    Step::backward, Step::left, Step::forward, Step::score,
    Step::backward, Step::right, Step::forward, Step::grab,
    Step::backward, Step::left, Step::forward, Step::score,
};

std::atomic<bool> mirror_selected{false};

double average_encoder_ticks(const std::vector<pros::Motor>& motors) {
    double total = 0.0;
    for (const auto& motor : motors) {
        const auto ticks = motor.get_raw_position(nullptr);
        if (ticks == PROS_ERR) {
            return PROS_ERR_F;
        }
        total += ticks;
    }
    return total / motors.size();
}

WheelTravel encoder_ticks() {
    return {average_encoder_ticks(chassis.left_motors), average_encoder_ticks(chassis.right_motors)};
}

bool valid(WheelTravel position) {
    return std::isfinite(position.left) && std::isfinite(position.right);
}

WheelTravel straight_target(double inches) {
    return {inches, inches};
}

WheelTravel turn_target(double degrees) {
    const double travel = std::numbers::pi * track_width_inches * degrees / 360.0;
    return {travel, -travel};
}

int wheel_power(ez::PID& pid, double travel, int max_power) {
    const double power = pid.compute(travel);
    if (std::abs(pid.target_get() - travel) <= position_tolerance_inches) {
        return 0;
    }
    return static_cast<int>(std::clamp(power, -static_cast<double>(max_power),
                                       static_cast<double>(max_power)));
}

bool start_encoder_move(EncoderMove& move, WheelTravel target, DriveSettings settings) {
    move.start = encoder_ticks();
    move.ticks_per_inch = chassis.drive_tick_per_inch();
    if (!valid(move.start) || !std::isfinite(move.ticks_per_inch) || move.ticks_per_inch <= 0.0) {
        chassis.drive_set(0, 0);
        std::puts("Invalid drive encoder reading");
        return false;
    }

    move.left_pid.variables_reset();
    move.right_pid.variables_reset();
    move.left_pid.constants_set(settings.kP, 0.0, settings.kD);
    move.right_pid.constants_set(settings.kP, 0.0, settings.kD);
    move.left_pid.target_set(target.left);
    move.right_pid.target_set(target.right);
    move.max_power = settings.max_power;
    move.started_at = pros::millis();
    move.settled_at.reset();
    std::printf("Encoder targets: left %.2f in, right %.2f in\n", target.left, target.right);
    return true;
}

MoveResult update_encoder_move(EncoderMove& move) {
    const auto now = pros::millis();
    const auto position = encoder_ticks();
    if (!valid(position)) {
        chassis.drive_set(0, 0);
        std::puts("Invalid drive encoder reading");
        return MoveResult::failed;
    }

    const WheelTravel travel{(position.left - move.start.left) * distance_scale / move.ticks_per_inch,
                             (position.right - move.start.right) * distance_scale / move.ticks_per_inch};
    const bool in_tolerance =
        std::abs(move.left_pid.target_get() - travel.left) <= position_tolerance_inches &&
        std::abs(move.right_pid.target_get() - travel.right) <= position_tolerance_inches;
    if (!in_tolerance) {
        move.settled_at.reset();
    } else if (!move.settled_at) {
        move.settled_at = now;
    }

    const bool settled = move.settled_at && now - *move.settled_at >= settle_time_ms;
    if (settled || now - move.started_at >= timeout_ms) {
        chassis.drive_set(0, 0);
        const double reported_turn = (travel.left - travel.right) * 180.0 /
                                     (std::numbers::pi * track_width_inches);
        std::printf("Encoder move %s: left %.2f in, right %.2f in, turn %.1f deg\n",
                    settled ? "settled" : "timed out", travel.left, travel.right, reported_turn);
        return settled ? MoveResult::settled : MoveResult::failed;
    }

    chassis.drive_set(wheel_power(move.left_pid, travel.left, move.max_power),
                      wheel_power(move.right_pid, travel.right, move.max_power));
    return MoveResult::running;
}

bool run_encoder_move(WheelTravel target, DriveSettings settings) {
    EncoderMove move;
    if (!start_encoder_move(move, target, settings)) {
        return false;
    }
    while (autonomous_movement_allowed()) {
        const auto result = update_encoder_move(move);
        if (result != MoveResult::running) {
            return result == MoveResult::settled;
        }
        pros::delay(ez::util::DELAY_TIME);
    }
    chassis.drive_set(0, 0);
    return false;
}

bool run_step(Step step, bool mirrored) {
    const double right_turn_degrees = mirrored ? -90.0 : 90.0;
    switch (step) {
        case Step::backward:
            return run_encoder_move(straight_target(-backward_inches), straight_settings);
        case Step::forward:
            return run_encoder_move(straight_target(forward_inches), straight_settings);
        case Step::left:
            return run_encoder_move(turn_target(-right_turn_degrees), turn_settings);
        case Step::right:
            return run_encoder_move(turn_target(right_turn_degrees), turn_settings);
        case Step::score:
            return autonomous_score(lift_raise_motor_degrees);
        case Step::grab:
            return autonomous_grab();
    }
    return false;
}

void select_original() {
    mirror_selected = false;
    pros::lcd::set_text(0, "Auton: original (first turn left)");
}

void select_mirror() {
    mirror_selected = true;
    pros::lcd::set_text(0, "Auton: mirrored (first turn right)");
}

#ifdef RECF_ENCODER_TUNING
EncoderMove tuning_move;
bool tuning_active = false;
#endif

}  // namespace

void initialize_autonomous_selector() {
    if (pros::lcd::is_initialized() || pros::lcd::initialize()) {
        pros::lcd::register_btn0_cb(select_original);
        pros::lcd::register_btn2_cb(select_mirror);
        pros::lcd::set_text(1, "Left button: original   Right button: mirror");
        if (lift_raise_motor_degrees <= 0.0) {
            pros::lcd::set_text(2, "Auton blocked: set scoring lift height");
        } else {
            pros::lcd::set_text(2, "Standalone: hold controller A to run");
        }
        select_original();
    }
}

void run_autonomous_route() {
    chassis.drive_set(0, 0);
    if (!autonomous_movement_allowed()) {
        return;
    }
    if (lift_raise_motor_degrees <= 0.0) {
        std::puts("Autonomous blocked: set lift_raise_motor_degrees");
        return;
    }
    if (!prepare_autonomous_mechanisms()) {
        std::puts("Autonomous blocked: mechanism setup failed");
        return;
    }

    const bool mirrored = mirror_selected.load();
    for (std::size_t i = 0; i < std::size(route); ++i) {
        if (!autonomous_movement_allowed() || !run_step(route[i], mirrored)) {
            chassis.drive_set(0, 0);
            std::printf("Autonomous stopped at step %u\n", static_cast<unsigned>(i + 1));
            return;
        }
    }
}

#ifdef RECF_ENCODER_TUNING
void reset_encoder_tuning() {
    tuning_active = false;
    chassis.drive_set(0, 0);
}

bool update_encoder_tuning(pros::Controller& controller) {
    const bool armed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) == 1;
    const bool straight_tapped = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == 1;
    const bool turn_tapped = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1;

    if (tuning_active && !armed) {
        reset_encoder_tuning();
        std::puts("Encoder test cancelled");
        return true;
    }
    if (!tuning_active) {
        if (!armed || straight_tapped == turn_tapped) {
            return false;
        }
        tuning_active = turn_tapped ? start_encoder_move(tuning_move, turn_target(90.0), turn_settings)
                                    : start_encoder_move(tuning_move, straight_target(24.0), straight_settings);
    }
    if (tuning_active) {
        tuning_active = update_encoder_move(tuning_move) == MoveResult::running;
    }
    return true;
}
#endif

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
