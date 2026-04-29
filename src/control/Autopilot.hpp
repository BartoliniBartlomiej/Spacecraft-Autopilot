// src/control/Autopilot.hpp
#pragma once

#include "control/ControlStrategy.hpp"
#include "control/PIDController.hpp"
#include "core/State.hpp"
#include "core/Diagnostics.hpp"
#include "core/Spacecraft.hpp"
#include "physics/PhysicsModel.hpp"
#include <optional>

class Autopilot : public ControlStrategy {
public:
    struct Config {
        PIDController::Gains vertical_gains;
        PIDController::Gains horizontal_gains;
        double target_x   = 0.0;
        double target_vy  = 0.0;
    };

    explicit Autopilot(const Spacecraft& spacecraft, Config config);

    [[nodiscard]] ThrustCommand compute(const State& state, double dt) override;
    void reset() override;

    [[nodiscard]] std::optional<ControlDiagnostics> last_diagnostics() const override;
    

private:
    const Spacecraft& m_spacecraft;
    Config           m_config;
    PIDController    m_vertical_pid;
    PIDController    m_horizontal_pid;
    ControlDiagnostics m_diagnostics;
};