#ifndef BATCH_SIMULATOR_HPP
#define BATCH_SIMULATOR_HPP

#include <iostream>
#include <format>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

#include "core/SimulationEngine.hpp"
#include "control/Autopilot.hpp"

class BatchSimulator {
public:
    static void runGridSearch(double step, double max_val) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Base configuration for the simulation
        SimulationEngine::Config sim_config;
        sim_config.timestep             = 0.01;
        sim_config.time_limit           = 600.0;
        sim_config.max_landing_velocity = 2.0;

        State initial;
        initial.x    =  0.0;
        initial.y    = 500.0;
        initial.vx   =  0.0;
        initial.vy   = -50.0;
        initial.mass =  500.0;

        int total_runs = 0;
        int successes  = 0;

        std::cout << std::format("\n[BATCH] Starting Grid Search | Step: {} | Max: {}\n", step, max_val);
        std::cout << std::string(55, '-') << "\n";
        std::cout << std::format("{:>7} | {:>7} | {:>7} | {:>12}\n", "Kp", "Ki", "Kd", "Result");
        std::cout << std::string(55, '-') << "\n";

        // Grid Search
        for (double kp = 0.0; kp <= max_val; kp += step) {
            for (double ki = 0.0; ki <= max_val; ki += step) {
                for (double kd = 0.0; kd <= max_val; kd += step) {
                    
                    Autopilot::Config autopilot_config;
                    autopilot_config.vertical_gains   = {kp, ki, kd};
                    autopilot_config.horizontal_gains = {10.0, 0.5, 0.5};
                    autopilot_config.max_thrust       = 15000.0;
                    autopilot_config.target_vy        = -1.5;
                    autopilot_config.target_x         = 0.0;

                    SimulationEngine engine{
                        sim_config,
                        std::make_unique<Autopilot>(autopilot_config),
                        PhysicsModel{9.81, 0.01}
                    };

                    SimulationEngine::Status result = engine.run(initial);
                    total_runs++;

                    if (result == SimulationEngine::Status::Landed) {
                        successes++;
                        std::cout << std::format("{:7.1f} | {:7.1f} | {:7.1f} | {:>12}\n", 
                                                 kp, ki, kd, "SUCCESS");
                    }
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

        std::cout << std::string(55, '-') << "\n";
        double rate = (total_runs > 0) ? (static_cast<double>(successes) / total_runs) * 100.0 : 0.0;
        
        std::cout << std::format("Total Runs:   {}\n", total_runs);
        std::cout << std::format("Successes:    {}\n", successes);
        std::cout << std::format("Success Rate: {:.2f}%\n", rate);
        std::cout << std::format("Execution Time: {:.3f} seconds\n", elapsed.count());
        std::cout << std::string(55, '-') << "\n\n";
    }
};

#endif