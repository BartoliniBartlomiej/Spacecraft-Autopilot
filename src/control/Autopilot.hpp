// Autopilot.hpp

#pragma once

#include "control/ControlStrategy.hpp"
#include "control/PIDController.hpp"
#include "core/State.hpp"
#include "physics/PhysicsModel.hpp"

#include <memory>

class Autopilot : public ControlStrategy {
public:
    struct Config {
        PIDController::Gains vertical_gains;
        PIDController::Gains horizontal_gains;
        double max_thrust = 8000.0; // [N]
        double target_x = 0.0; // [m]
        double target_vy = 0.0; // [m/s]
    };

    struct Diagnostics {
        double vertical_error   = 0.0;
        double horizontal_error = 0.0;
        double vertical_output  = 0.0;
        double horizontal_output = 0.0;
        PIDController::Gains vertical_gains;
        PIDController::Gains horizontal_gains;
    };

    explicit Autopilot(Config config);

    [[nodiscard]] ThrustCommand compute(const State& state, double dt) override;
    void reset() override;

    [[nodiscard]] const Diagnostics& last_diagnostics() const;

private:
    Config m_config;
    PIDController m_vertical_pid;
    PIDController m_horizontal_pid;
    Diagnostics m_diagnostics;
};