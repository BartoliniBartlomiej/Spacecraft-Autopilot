// Autopilot.cpp

#include "control/Autopilot.hpp"
#include <iostream>
#include <algorithm>

Autopilot::Autopilot(Config config) :
    m_config{config},
    m_vertical_pid{config.vertical_gains, 0.0, config.max_thrust},
    m_horizontal_pid{config.horizontal_gains, -config.max_thrust * 0.3, config.max_thrust * 0.3}
{}

ThrustCommand Autopilot::compute(const State& state, double dt)
{
    const double vertical_error   = m_config.target_vy - state.vy;
    const double horizontal_error = m_config.target_x  - state.x;

    // Feedforward — gravitation compensation (F = mg)
    const double gravity_compensation = 9.81 * state.mass;

    ThrustCommand cmd;
    cmd.fy = gravity_compensation + m_vertical_pid.compute(vertical_error, dt);
    cmd.fx = m_horizontal_pid.compute(horizontal_error, dt);

    // Clamp to max_thrust
    cmd.fy = std::clamp(cmd.fy, 0.0, m_config.max_thrust);

    return cmd;
}

void Autopilot::reset() {
    m_vertical_pid.reset();
    m_horizontal_pid.reset();
}