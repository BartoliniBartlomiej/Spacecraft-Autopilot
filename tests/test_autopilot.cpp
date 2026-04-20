#include <gtest/gtest.h>
#include "control/Autopilot.hpp"

TEST(AutopilotTest, ThrustUpWhenFallingTooFast) {
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 2.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.max_thrust       = 8000.0;
    config.target_vy        = 0.0;
    config.target_x         = 0.0;

    Autopilot autopilot{config};

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
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 2.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.max_thrust       = 8000.0;
    config.target_vy        = 0.0;
    config.target_x         = 0.0;

    Autopilot autopilot{config};

    State state;
    state.vy   = 0.0;   // velocity == target velocity
    state.vx   = 0.0;
    state.x    = 0.0;
    state.mass = 500.0;

    const ThrustCommand cmd = autopilot.compute(state, 0.1);

    EXPECT_NEAR(cmd.fy, 0.0, 1e-9);
    EXPECT_NEAR(cmd.fx, 0.0, 1e-9);
}

TEST(AutopilotTest, HorizontalCorrectionWhenOffTarget) {
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 0.0, .ki = 0.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 1.0, .ki = 0.0, .kd = 0.0};
    config.max_thrust       = 8000.0;
    config.target_x         = 0.0;

    Autopilot autopilot{config};

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
    Autopilot::Config config;
    config.vertical_gains   = {.kp = 0.0, .ki = 1.0, .kd = 0.0};
    config.horizontal_gains = {.kp = 0.0, .ki = 0.0, .kd = 0.0};
    config.max_thrust       = 8000.0;
    config.target_vy        = 0.0;

    Autopilot autopilot{config};

    State state;
    state.vy   = -10.0;
    state.mass = 500.0;

    autopilot.compute(state, 1.0);  // integral accumulates
    autopilot.reset();

    state.vy = 0.0;  // error = 0 after reset
    const ThrustCommand cmd = autopilot.compute(state, 1.0);

    // Integral = 0 after reset -> output = 0
    EXPECT_NEAR(cmd.fy, 0.0, 1e-9);
}