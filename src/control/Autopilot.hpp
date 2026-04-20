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

    explicit Autopilot(Config config);

    [[nodiscard]] ThrustCommand compute(const State& current_state, double dt) override;
    void reset() override;

public:
    Config m_config;
    PIDController m_vertical_pid;
    PIDController m_horizontal_pid;
};