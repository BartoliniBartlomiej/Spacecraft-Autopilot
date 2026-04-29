#include "multisimulation/BatchSimulation.hpp"
#include "services/DataLogger.hpp"
#include <iostream>
#include <format>
#include <chrono>

std::vector<BatchSimulator::RunResult> BatchSimulator::run(const std::vector<Scenario>& scenarios, bool save_reports) {
    std::vector<RunResult> results;
    DataLogger logger;

    for (const auto& scenario : scenarios) {
        if (!scenario.spacecraft) continue;

        auto start = std::chrono::steady_clock::now();

        SimulationEngine engine{
            *scenario.spacecraft,
            scenario.sim_config,
            std::make_unique<Autopilot>(*scenario.spacecraft, scenario.autopilot_config),
            PhysicsModel{*scenario.spacecraft}
        };

        const auto status = engine.run(scenario.initial_state);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = end - start;

        results.push_back({scenario.label, status, diff.count()});

        if (save_reports) {
            std::string filename = std::format("output_data/batch_{}.csv", scenario.label);
            logger.saveReport(filename, engine);
        }
    }

    return results;
}

std::vector<BatchSimulator::Scenario> BatchSimulator::buildGridSearch(
    std::shared_ptr<Spacecraft> spacecraft,
    double kp_max, double ki_max, double kd_max, double step,
    const State& initial_state,
    const SimulationEngine::Config& sim_config,
    const Autopilot::Config& base_autopilot_config) 
{
    std::vector<Scenario> scenarios;

    for (double kp = 0.0; kp <= kp_max; kp += step) {
        for (double ki = 0.0; ki <= ki_max; ki += step) {
            for (double kd = 0.0; kd <= kd_max; kd += step) {
                Scenario s;
                s.spacecraft = spacecraft;
                s.initial_state = initial_state;
                s.sim_config = sim_config;
                s.autopilot_config = base_autopilot_config;
                s.autopilot_config.vertical_gains = {kp, ki, kd};
                s.label = std::format("PID_{:.1f}_{:.1f}_{:.1f}", kp, ki, kd);
                scenarios.push_back(s);
            }
        }
    }

    return scenarios;
}
