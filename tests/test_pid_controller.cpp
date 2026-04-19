#include <gtest/gtest.h>
#include "control/PIDController.hpp"

TEST(PIDControllerTest, ProportionalOnly){
    PIDController pid{{.kp = 2.0, .ki = 0.0, .kd = 0.0}, -100.0, 100.0};

    // Error 5.0 → output = kp * error = 2.0 * 5.0 = 10.0
    const double output = pid.compute(5.0, 0.1);
    EXPECT_NEAR(output, 10.0, 1e-9);
}

TEST(PIDControllerTest, IntegralAccumulates) {
    PIDController pid{{.kp = 0.0, .ki = 1.0, .kd = 0.0}, -100.0, 100.0};

    // Error 1.0 for 3 steps with dt=1.0 → integral = 3.0, output = 3.0
    pid.compute(1.0, 1.0);
    pid.compute(1.0, 1.0);
    const double output = pid.compute(1.0, 1.0);

    EXPECT_NEAR(output, 3.0, 1e-9);
}

TEST(PIDControllerTest, DerivativeOnErrorChange) {
    PIDController pid{{.kp = 0.0, .ki = 0.0, .kd = 1.0}, -100.0, 100.0};

    // First step — no previous error, derivative = 0
    pid.compute(0.0, 0.1);

    // Error jumps from 0.0 to 10.0 → derivative = (10.0 - 0.0) / 0.1 = 100.0
    const double output = pid.compute(10.0, 0.1);
    EXPECT_NEAR(output, 100.0, 1e-9);
}

TEST(PIDControllerTest, OutputClampedToLimits) {
    PIDController pid{{.kp = 100.0, .ki = 0.0, .kd = 0.0}, -50.0, 50.0};

    // kp * error = 100 * 10 = 1000 — but limited to 50
    const double output = pid.compute(10.0, 0.1);
    EXPECT_NEAR(output, 50.0, 1e-9);
}

TEST(PIDControllerTest, ResetClearsState) {
    PIDController pid{{.kp = 0.0, .ki = 1.0, .kd = 0.0}, -100.0, 100.0};

    pid.compute(5.0, 1.0);  // integral = 5.0
    pid.reset();
    const double output = pid.compute(1.0, 1.0);  // after reset integral starts from 0

    EXPECT_NEAR(output, 1.0, 1e-9);
}