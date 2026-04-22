// src/core/SimulationEngine.hpp
#pragma once

#include "core/State.hpp"
#include "core/Diagnostics.hpp"
#include "physics/PhysicsModel.hpp"
#include "control/ControlStrategy.hpp"

#include <memory>
#include <functional>
#include <vector>
#include <string>

class SimulationEngine {
public:
    struct Config {
        double timestep             = 0.01;
        double time_limit           = 100.0;
        double landing_altitude     = 0.0;
        double max_landing_velocity = 2.0;
    };

    enum class Status {
        Running,
        Landed,
        Crashed,
        FuelExhausted,
        TimeLimitReached
    };

    using StepCallback = std::function<void(const State&,
                                            const ThrustCommand&,
                                            double time,
                                            Status)>;

    SimulationEngine(Config config,
                     std::unique_ptr<ControlStrategy> autopilot,
                     PhysicsModel physics);

    Status run(State initial_state);
    Status step(State& state, double& time);

    void set_step_callback(StepCallback callback);
    void saveReport(const std::string& filename) const;

private:
    [[nodiscard]] Status check_status(const State& state) const;
    [[nodiscard]] State  integrate(const State& state, const ThrustCommand& cmd) const;
    // [[nodiscard]] std::string statusToString(Status status) const;

    struct StepRecord {
        double             time;
        State              state;
        ThrustCommand      cmd;
        ControlDiagnostics diagnostics;
    };

    Config                           m_config;
    std::unique_ptr<ControlStrategy> m_autopilot;
    PhysicsModel                     m_physics;
    StepCallback                     m_callback;
    Status                           m_lastStatus = Status::Running;
    std::vector<StepRecord>          m_history;

    std::string statusToString(SimulationEngine::Status status) const {
        switch (status) {
            case SimulationEngine::Status::Running:       return "Running";
            case SimulationEngine::Status::Landed:        return "Landed";
            case SimulationEngine::Status::Crashed:       return "Crashed";
            case SimulationEngine::Status::FuelExhausted: return "FuelExhausted";
            case SimulationEngine::Status::TimeLimitReached: return "TimeLimitReached";
            default:                                      return "Unknown";
        }
    }
};