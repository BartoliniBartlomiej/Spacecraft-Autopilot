// src/multisimulation/BatchSimulation.cpp
#include "multisimulation/BatchSimulation.hpp"

#include <iostream>
#include <format>
#include <chrono>

#include "services/DataLogger.hpp"

std::vector<BatchSimulator::RunResult> BatchSimulator::run(const std::vector<Scenario>& scenarios, bool save_reports) {
    std::vector<RunResult> results;

    DataLogger logger;

    results.reserve(scenarios.size());

    const auto total_start = std::chrono::high_resolution_clock::now();

    std::cout << std::format("\n[BATCH] Running {} scenarios\n", scenarios.size());
    std::cout << std::string(55, '-') << "\n";
    std::cout << std::format("{:<20} | {:<14} | {:<10}\n", "Label", "Status", "Time [s]");
    std::cout << std::string(55, '-') << "\n";

    for (const auto& scenario : scenarios) {
        SimulationEngine engine{
            scenario.sim_config,
            std::make_unique<Autopilot>(scenario.autopilot_config),
            PhysicsModel{}
        };

        const auto run_start = std::chrono::high_resolution_clock::now();
        const auto status    = engine.run(scenario.initial_state);
        const auto run_end   = std::chrono::high_resolution_clock::now();

        const double elapsed =
            std::chrono::duration<double>(run_end - run_start).count();

        if (save_reports){
            logger.saveReport("output_data/batch_" + scenario.label + ".csv", engine, true);

        }

        const std::string status_str = [&] {
            switch (status) {
                case SimulationEngine::Status::Landed:           return "Landed";
                case SimulationEngine::Status::Crashed:          return "Crashed";
                case SimulationEngine::Status::FuelExhausted:    return "FuelExhausted";
                case SimulationEngine::Status::TimeLimitReached: return "TimeLimitReached";
                default:                                         return "Unknown";
            }
        }();

        std::cout << std::format("{:<20} | {:<14} | {:.3f}\n",
            scenario.label, status_str, elapsed);

        results.push_back({scenario.label, status, elapsed});
    }

    const auto total_end = std::chrono::high_resolution_clock::now();
    const double total   =
        std::chrono::duration<double>(total_end - total_start).count();

    const long successes = std::count_if(results.begin(), results.end(),
        [](const RunResult& r) {
            return r.status == SimulationEngine::Status::Landed;
        });

    std::cout << std::string(55, '-') << "\n";
    std::cout << std::format("Successes:      {}/{}\n", successes, results.size());
    std::cout << std::format("Success rate:   {:.1f}%\n",
        100.0 * static_cast<double>(successes) / static_cast<double>(results.size()));
    std::cout << std::format("Total time:     {:.3f} s\n", total);
    std::cout << std::string(55, '-') << "\n";

    return results;
}

std::vector<BatchSimulator::Scenario> BatchSimulator::buildGridSearch(
    double kp_max, double ki_max, double kd_max, double step,
    const State& initial_state,
    const SimulationEngine::Config& sim_config,
    const Autopilot::Config& base_autopilot_config)
{
    std::vector<Scenario> scenarios;

    for (double kp = 0.0; kp <= kp_max; kp += step)
        for (double ki = 0.0; ki <= ki_max; ki += step)
            for (double kd = 0.0; kd <= kd_max; kd += step) {
                Autopilot::Config cfg  = base_autopilot_config;
                cfg.vertical_gains     = {kp, ki, kd};

                scenarios.push_back({
                    .initial_state    = initial_state,
                    .autopilot_config = cfg,
                    .sim_config       = sim_config,
                    .label            = std::format("kp{:.1f}_ki{:.1f}_kd{:.1f}", kp, ki, kd)
                });
            }

    return scenarios;
}