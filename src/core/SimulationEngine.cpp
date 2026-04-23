// SimulationEngine.cpp

#include "core/SimulationEngine.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <format>
#include <filesystem>

SimulationEngine::SimulationEngine(Config config,
                                   std::unique_ptr<ControlStrategy> autopilot,
                                   PhysicsModel physics) :
    m_config{config},
    m_autopilot{std::move(autopilot)},
    m_physics{physics}
{}

SimulationEngine::Status SimulationEngine::run(State initial_state) {
    m_history.clear();

    State  state  = initial_state;
    double time   = 0.0;
    Status status = Status::Running;

    while (status == Status::Running)
        status = step(state, time);

    m_lastStatus = status;

    return status;
}

SimulationEngine::Status SimulationEngine::step(State& state, double& time) {
    const ThrustCommand cmd = m_autopilot->compute(state, m_config.timestep);

    ControlDiagnostics diag;
    if (auto opt = m_autopilot->last_diagnostics(); opt.has_value())
        diag = *opt;

    m_history.push_back({time, state, cmd, diag});

    state = integrate(state, cmd);
    time += m_config.timestep;

    if (time >= m_config.time_limit)
        return Status::TimeLimitReached;

    const Status status = check_status(state);

    if (m_callback)
        m_callback(state, cmd, time, status);

    return status;
}

void SimulationEngine::set_step_callback(StepCallback callback) {
    m_callback = std::move(callback);
}

SimulationEngine::Status SimulationEngine::check_status(const State& state) const {
    // Time limit reached
    if (state.mass <= 300.0)
        return Status::FuelExhausted;

    // Eagle (Spacecraft) has landed 
    if (state.y <= m_config.landing_altitude)
    {
        const double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);

        if (speed <= m_config.max_landing_velocity)
            return Status::Landed;
        else
            return Status::Crashed;
    }

    return Status::Running;
}

State SimulationEngine::integrate(const State& state, const ThrustCommand& cmd) const {
    const double dt = m_config.timestep;

    // RK4
    const State k1 = m_physics.derivative(state, cmd);

    State s2 = state;
    s2.x  += k1.x  * dt / 2.0;
    s2.y  += k1.y  * dt / 2.0;
    s2.vx += k1.vx * dt / 2.0;
    s2.vy += k1.vy * dt / 2.0;
    s2.mass += k1.mass * dt / 2.0;
    const State k2 = m_physics.derivative(s2, cmd);

    State s3 = state;
    s3.x  += k2.x  * dt / 2.0;
    s3.y  += k2.y  * dt / 2.0;
    s3.vx += k2.vx * dt / 2.0;
    s3.vy += k2.vy * dt / 2.0;
    s3.mass += k2.mass * dt / 2.0;
    const State k3 = m_physics.derivative(s3, cmd);

    State s4 = state;
    s4.x  += k3.x  * dt;
    s4.y  += k3.y  * dt;
    s4.vx += k3.vx * dt;
    s4.vy += k3.vy * dt;
    s4.mass += k3.mass * dt;
    const State k4 = m_physics.derivative(s4, cmd);

    State next = state;
    next.x  += (k1.x  + 2*k2.x  + 2*k3.x  + k4.x)  * dt / 6.0;
    next.y  += (k1.y  + 2*k2.y  + 2*k3.y  + k4.y)  * dt / 6.0;
    next.vx += (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx) * dt / 6.0;
    next.vy += (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy) * dt / 6.0;
    next.mass += (k1.mass + 2*k2.mass + 2*k3.mass + k4.mass) * dt / 6.0;

    return next;
}

