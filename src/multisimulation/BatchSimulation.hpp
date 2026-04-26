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

    // Sweep range and step for a single set of PID gains (kp / ki / kd independently)
    struct GainRange {
        double kp_max  = 0.0;  double kp_step = 1.0;
        double ki_max  = 0.0;  double ki_step = 1.0;
        double kd_max  = 0.0;  double kd_step = 1.0;
    };

    // Run a pre-built list of scenarios and return results
    [[nodiscard]] static std::vector<RunResult> run(const std::vector<Scenario>& scenarios,bool save_reports = false);

    // Build scenarios by independently grid-searching vertical and horizontal PID gains
    [[nodiscard]] static std::vector<Scenario> buildGridSearch(
        const GainRange& vertical,
        const GainRange& horizontal,
        const State& initial_state,
        const SimulationEngine::Config& sim_config,
        const Autopilot::Config& base_autopilot_config);
};