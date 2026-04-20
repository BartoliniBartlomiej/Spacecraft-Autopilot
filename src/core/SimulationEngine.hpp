// SimulationEngine.hpp

#pragma once

#include "core/State.hpp"
#include "physics/PhysicsModel.hpp"
#include "control/ControlStrategy.hpp"

#include <memory>
#include <functional>

class SimulationEngine {
public:
    struct Config {
        double timestep = 0.01; // [s]
        double time_limit = 100.0; // [s]
        double landing_altitude = 0.0; // [m]
        double max_landing_velocity = 2.0; // [m/s]
    };

    enum class Status {
        Running,
        Landed,
        Crashed,
        FuelExhausted,
        TimeLimitReached
    };

    using StepCallback = std::function<void(const State&, double time, Status)>;

    SimulationEngine(Config config,
                     std::unique_ptr<ControlStrategy> autopilot,
                     PhysicsModel physics);

    Status run(State initial_state); // run full simulation thru the end

    Status step(State& state, double& time); // single step - (for render???)

    void set_step_callback(StepCallback callback);

private:
    [[nodiscard]] Status check_status(const State& state) const;
    [[nodiscard]] State integrate(const State& state, const ThrustCommand& cmd) const;

    Config m_config;
    std::unique_ptr<ControlStrategy> m_autopilot;
    PhysicsModel m_physics;
    StepCallback m_callback;
};

