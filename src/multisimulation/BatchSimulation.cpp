// src/multisimulation/BatchSimulation.cpp
#include "multisimulation/BatchSimulation.hpp"

#include <iostream>
#include <format>
#include <chrono>

#include "services/DataLogger.hpp"

std::vector<BatchSimulator::RunResult> BatchSimulator::run(
    const std::vector<Scenario>& scenarios, bool save_reports)
{
    std::vector<RunResult> results;
    results.reserve(scenarios.size());

    DataLogger logger;

    const auto total_start = std::chrono::high_resolution_clock::now();

    std::cout << std::format("\n[BATCH] Running {} scenarios\n", scenarios.size());
    std::cout << std::string(55, '-') << "\n";
    std::cout << std::format("{:<20} | {:<14} | {:<10}\n", "Label", "Status", "Time [s]");
    std::cout << std::string(55, '-') << "\n";

    for (const auto& scenario : scenarios)
    {
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

        if (save_reports)
            logger.saveReport("output_data/batch_" + scenario.label + ".csv", engine);

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
    const GainRange& vertical,
    const GainRange& horizontal,
    const State& initial_state,
    const SimulationEngine::Config& sim_config,
    const Autopilot::Config& base_autopilot_config)
{
    std::vector<Scenario> scenarios;

    for (double v_kp = 0.0; v_kp <= vertical.kp_max;   v_kp += vertical.kp_step)
    for (double v_ki = 0.0; v_ki <= vertical.ki_max;   v_ki += vertical.ki_step)
    for (double v_kd = 0.0; v_kd <= vertical.kd_max;   v_kd += vertical.kd_step)
    for (double h_kp = 0.0; h_kp <= horizontal.kp_max; h_kp += horizontal.kp_step)
    for (double h_ki = 0.0; h_ki <= horizontal.ki_max; h_ki += horizontal.ki_step)
    for (double h_kd = 0.0; h_kd <= horizontal.kd_max; h_kd += horizontal.kd_step)
    {
        Autopilot::Config cfg = base_autopilot_config;
        cfg.vertical_gains    = {v_kp, v_ki, v_kd};
        cfg.horizontal_gains  = {h_kp, h_ki, h_kd};

        scenarios.push_back({
            .initial_state    = initial_state,
            .autopilot_config = cfg,
            .sim_config       = sim_config,
            .label            = std::format(
                "v({:.1f},{:.1f},{:.1f})_h({:.1f},{:.1f},{:.1f})",
                v_kp, v_ki, v_kd, h_kp, h_ki, h_kd)
        });
    }

    return scenarios;
}