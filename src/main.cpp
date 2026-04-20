// main.cpp

#include <iostream>
#include <format>

#include "core/SimulationEngine.hpp"
#include "control/Autopilot.hpp"

int main()
{
    // Autopilot configuration
    Autopilot::Config autopilot_config;
    autopilot_config.vertical_gains   = {.kp = 80.0, .ki = 0.5, .kd = 30.0};
    autopilot_config.horizontal_gains = {.kp = 1.0,  .ki = 0.0, .kd = 0.0};
    autopilot_config.max_thrust       = 15000.0;
    autopilot_config.target_vy        = -1.5;
    autopilot_config.target_x         = 0.0;

    // Simulation engine configuration
    SimulationEngine::Config sim_config;
    sim_config.timestep             = 0.01;
    sim_config.time_limit           = 600.0;
    sim_config.max_landing_velocity = 2.0;

    SimulationEngine engine{
        sim_config,
        std::make_unique<Autopilot>(autopilot_config),
        PhysicsModel{9.81, 0.05}
    };

    // Initial state
    State initial;
    initial.x    =  10.0;   // 10m on the right of the target
    initial.y    = 500.0;   // 500m height
    initial.vx   =  2.0;    // slight lateral drift
    initial.vy   = -20.0;   // falls 20 m/s
    initial.mass =  500.0;  // 500 kg

    double last_log = 0.0;
    engine.set_step_callback([&last_log](const State& s, double time, SimulationEngine::Status status)
    {
        if (time - last_log >= 1.0 || status != SimulationEngine::Status::Running)
        {
            std::cout << std::format(
                "t={:6.1f}s  y={:7.1f}m  vy={:6.2f}m/s  x={:6.1f}m  vx={:5.2f}m/s\n",
                time, s.y, s.vy, s.x, s.vx);
            last_log = time;
        }
    });
    
    const auto result = engine.run(initial);

    switch (result)
    {
        case SimulationEngine::Status::Landed:
            std::cout << "\n✓ Soft landing successful!\n"; break;
        case SimulationEngine::Status::Crashed:
            std::cout << "\n✗ Crashed — velocity too high!\n"; break;
        case SimulationEngine::Status::FuelExhausted:
            std::cout << "\n✗ Fuel exhausted!\n"; break;
        default:
            std::cout << "\n✗ Time limit reached!\n"; break;
    }

    return 0;
}