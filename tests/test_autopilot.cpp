#include <gtest/gtest.h>
#include <tuple>
#include "control/Autopilot.hpp"
#include "core/Spacecraft.hpp"

static Spacecraft createTestSpacecraft(double max_thrust = 15000.0) {
    Spacecraft s;
    s.dry_mass = 300.0;
    s.fuel_capacity = 200.0;
    s.thrusters = {{"Main", max_thrust, 0, 0, 0, 1}};
    return s;
}

TEST(AutopilotTest, ThrustUpWhenFallingTooFast) {
    Spacecraft sc = createTestSpacecraft(8000.0);
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 2.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.target_vy        = 0.0;
    config.target_x         = 0.0;

    Autopilot autopilot{sc, config};

    State state;
    state.vy = -50.0;  // falls too quick
    state.vx = 0.0;
    state.x  = 0.0;
    state.mass = 500.0;

    const ThrustCommand cmd = autopilot.compute(state, 0.1);

    // Velocity on minus -> error on plus -> thrust up
    EXPECT_GT(cmd.fy, 0.0);
}

TEST(AutopilotTest, NoThrustWhenVelocityOnTarget) {
    Spacecraft sc = createTestSpacecraft(8000.0);
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 2.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.target_vy        = 0.0;
    config.target_x         = 0.0;

    Autopilot autopilot{sc, config};

    State state;
    state.vy   = 0.0;
    state.vx   = 0.0;
    state.x    = 0.0;
    state.mass = 500.0;

    const ThrustCommand cmd = autopilot.compute(state, 0.1);

    // Error = 0, so PID = 0 -> only gravity compensation
    const double expected_fy = 9.81 * state.mass;
    EXPECT_NEAR(cmd.fy, expected_fy, 1e-6);
    EXPECT_NEAR(cmd.fx, 0.0, 1e-9);
}

TEST(AutopilotTest, HorizontalCorrectionWhenOffTarget) {
    Spacecraft sc = createTestSpacecraft(8000.0);
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 0.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.target_x         = 0.0;

    Autopilot autopilot{sc, config};

    State state;
    state.x    = 100.0;  // 100 m on the right of the target
    state.vy   = 0.0;
    state.vx   = 0.0;
    state.mass = 500.0;

    const ThrustCommand cmd = autopilot.compute(state, 0.1);

    // Spacecraft too far on the right -> thrust to the left (-fx)
    EXPECT_LT(cmd.fx, 0.0);
}

TEST(AutopilotTest, ResetClearsPIDState) {
    Spacecraft sc = createTestSpacecraft(100000.0);
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 0.0, .ki = 1.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 0.0, .ki = 0.0, .kd = 0.0};
    config.target_vy        = 0.0;

    Autopilot autopilot{sc, config};

    State state;
    state.vy   = -10.0;
    state.mass = 500.0;

    std::ignore = autopilot.compute(state, 1.0);  // integral accumulates
    autopilot.reset();

    state.vy = 0.0;  // error = 0 after reset
    const ThrustCommand cmd = autopilot.compute(state, 1.0);

    // After reset, integral should be cleared, so no extra thrust from it. Only gravity compensation.
    const double expected_fy = 9.81 * state.mass;
    EXPECT_NEAR(cmd.fy, expected_fy, 1e-6);
}
