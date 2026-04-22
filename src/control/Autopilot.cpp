// src/control/Autopilot.cpp
#include "control/Autopilot.hpp"
#include <algorithm>

Autopilot::Autopilot(Config config)
    : m_config{config}
    , m_vertical_pid{config.vertical_gains, 0.0, config.max_thrust}
    , m_horizontal_pid{config.horizontal_gains,
                       -config.max_thrust * 0.3,
                        config.max_thrust * 0.3}
{}

ThrustCommand Autopilot::compute(const State& state, double dt) {
    const double vertical_error   = m_config.target_vy - state.vy;
    const double horizontal_error = m_config.target_x  - state.x;
    const double gravity_compensation = 9.81 * state.mass;

    const double vertical_out   = m_vertical_pid.compute(vertical_error,   dt);
    const double horizontal_out = m_horizontal_pid.compute(horizontal_error, dt);

    m_diagnostics.vertical_error    = vertical_error;
    m_diagnostics.horizontal_error  = horizontal_error;
    m_diagnostics.vertical_output   = vertical_out;
    m_diagnostics.horizontal_output = horizontal_out;
    m_diagnostics.vertical_gains    = m_config.vertical_gains;
    m_diagnostics.horizontal_gains  = m_config.horizontal_gains;
    
    ThrustCommand cmd;
    cmd.fy = std::clamp(gravity_compensation + vertical_out, 0.0, m_config.max_thrust);
    cmd.fx = horizontal_out;

    return cmd;
}

std::optional<ControlDiagnostics> Autopilot::last_diagnostics() const {
    return m_diagnostics;
}

void Autopilot::reset() {
    m_vertical_pid.reset();
    m_horizontal_pid.reset();
}