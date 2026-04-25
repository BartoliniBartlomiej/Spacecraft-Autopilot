// src/multisimulation/BatchSimulation.hpp
#pragma once

#include "core/SimulationEngine.hpp"
#include "core/State.hpp"
#include "control/Autopilot.hpp"
#include <vector>
#include <string>

class BatchSimulator {
public:
    // Describes a single simulation scenario to run
    struct Scenario {
        State                    initial_state;
        Autopilot::Config        autopilot_config;
        SimulationEngine::Config sim_config;
        std::string              label; // optional name for logging
    };

    // Result of a single run
    struct RunResult {
        std::string              label;
        SimulationEngine::Status status;
        double                   duration_s;
    };

    // Run a pre-built list of scenarios and return results
    [[nodiscard]] static std::vector<RunResult> run(
        const std::vector<Scenario>& scenarios,
        bool save_reports = false);

    // Build scenarios by grid-searching vertical PID gains
    [[nodiscard]] static std::vector<Scenario> buildGridSearch(
        double kp_max, double ki_max, double kd_max, double step,
        const State& initial_state,
        const SimulationEngine::Config& sim_config,
        const Autopilot::Config& base_autopilot_config);
};