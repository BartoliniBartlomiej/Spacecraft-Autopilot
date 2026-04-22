// src/control/Autopilot.hpp
#pragma once

#include "control/ControlStrategy.hpp"
#include "control/PIDController.hpp"
#include "core/State.hpp"
#include "core/Diagnostics.hpp"
#include "physics/PhysicsModel.hpp"
#include <optional>

class Autopilot : public ControlStrategy {
public:
    struct Config {
        PIDController::Gains vertical_gains;
        PIDController::Gains horizontal_gains;
        double max_thrust = 8000.0;
        double target_x   = 0.0;
        double target_vy  = 0.0;
    };

    explicit Autopilot(Config config);

    [[nodiscard]] ThrustCommand compute(const State& state, double dt) override;
    void reset() override;

    [[nodiscard]] std::optional<ControlDiagnostics> last_diagnostics() const override;
    

private:
    Config           m_config;
    PIDController    m_vertical_pid;
    PIDController    m_horizontal_pid;
    ControlDiagnostics m_diagnostics;
};