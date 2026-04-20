// #include <iostream>
// #include <format>
// #include "core/SimulationEngine.hpp"
// #include "control/Autopilot.hpp"

// #ifdef ENABLE_RENDERING
// #include "rendering/Renderer.hpp"
// #endif

// int main()
// {
//     Autopilot::Config autopilot_config;
//     autopilot_config.vertical_gains   = {.kp = 80.0, .ki = 0.5, .kd = 30.0};
//     autopilot_config.horizontal_gains = {.kp = 1.0,  .ki = 0.0, .kd = 0.5};
//     autopilot_config.max_thrust       = 8000.0;
//     autopilot_config.target_vy        = -1.5;
//     autopilot_config.target_x         = 0.0;

//     SimulationEngine::Config sim_config;
//     sim_config.timestep             = 0.01;
//     sim_config.time_limit           = 600.0;
//     sim_config.max_landing_velocity = 2.0;

//     SimulationEngine engine{
//         sim_config,
//         std::make_unique<Autopilot>(autopilot_config),
//         PhysicsModel{9.81, 0.01}
//     };

//     State initial;
//     initial.x    =  50.0;
//     initial.y    = 50.0;
//     initial.vx   =  2.0;
//     initial.vy   = -20.0;
//     initial.mass =  500.0;

// #ifdef ENABLE_RENDERING
//     Renderer::Config renderer_config;
//     renderer_config.width      = 1024;
//     renderer_config.height     = 768;
//     renderer_config.scale      = 1.0f;
//     renderer_config.max_thrust = autopilot_config.max_thrust;

//     Renderer renderer{renderer_config};
//     SimulationEngine::Status last_status = SimulationEngine::Status::Running;

//     engine.set_step_callback([&](const State& s,
//                                   const ThrustCommand& cmd,
//                                   double time,
//                                   SimulationEngine::Status status)
//     {
//         last_status = status;
//         renderer.handle_events();
//         renderer.draw(s, cmd, time, status);
//     });

//     engine.run(initial);

//     while (renderer.is_open())
//     {
//         renderer.handle_events();
//         renderer.draw({}, {}, 0.0, last_status);
//     }

// #else
//     double last_log = 0.0;
//     engine.set_step_callback([&](const State& s,
//                                   const ThrustCommand& cmd,
//                                   double time,
//                                   SimulationEngine::Status status)
//     {
//         if (time - last_log >= 10.0 || status != SimulationEngine::Status::Running)
//         {
//             const double thrust_pct = (cmd.fy / autopilot_config.max_thrust) * 100.0;
//             std::cout << std::format(
//                 "t={:6.1f}s  y={:7.1f}m  vy={:6.2f}m/s  x={:6.1f}m  vx={:5.2f}m/s  thrust={:5.1f}%\n",
//                 time, s.y, s.vy, s.x, s.vx, thrust_pct);
//             last_log = time;
//         }
//     });

//     const auto result = engine.run(initial);

//     switch (result)
//     {
//         case SimulationEngine::Status::Landed:
//             std::cout << "\n✓ Soft landing successful!\n"; break;
//         case SimulationEngine::Status::Crashed:
//             std::cout << "\n✗ Crashed — velocity too high!\n"; break;
//         case SimulationEngine::Status::FuelExhausted:
//             std::cout << "\n✗ Fuel exhausted!\n"; break;
//         default:
//             std::cout << "\n✗ Time limit reached!\n"; break;
//     }
// #endif


//     engine.saveRaport("output.csv");

//     return 0;
// }

#include <iostream>
#include <format>
#include "core/SimulationEngine.hpp"
#include "control/Autopilot.hpp"

#ifdef ENABLE_RENDERING
#include "rendering/Renderer.hpp"
#endif

int main()
{
    // --- MENU WYBORU ---
    std::cout << "--- Spacecraft Autopilot ---\n";
    std::cout << "1. Bez wizualizacji (logi w konsoli)\n";
    std::cout << "2. Z wizualizacja (okno SFML)\n";
    std::cout << "Wybierz opcje (1/2): ";
    
    int choice = 1;
    std::cin >> choice;

    bool use_renderer = (choice == 2);

    // Zabezpieczenie: jeśli wybrano wizualizację, ale skompilowano bez niej
#ifndef ENABLE_RENDERING
    if (use_renderer) {
        std::cout << "\n[OSTRZEŻENIE] Program skompilowano bez wsparcia dla SFML (ENABLE_RENDERING=OFF).\n";
        std::cout << "Wymuszam tryb bez wizualizacji.\n\n";
        use_renderer = false;
    }
#endif

    std::cout << "\nRozpoczynam symulacje...\n";

    // --- KONFIGURACJA SYMULACJI ---
    Autopilot::Config autopilot_config;
    autopilot_config.vertical_gains   = {.kp = 80.0, .ki = 0.5, .kd = 30.0};
    autopilot_config.horizontal_gains = {.kp = 1.0,  .ki = 0.0, .kd = 0.5};
    autopilot_config.max_thrust       = 8000.0;
    autopilot_config.target_vy        = -1.5;
    autopilot_config.target_x         = 0.0;

    SimulationEngine::Config sim_config;
    sim_config.timestep             = 0.01;
    sim_config.time_limit           = 600.0;
    sim_config.max_landing_velocity = 2.0;

    SimulationEngine engine{
        sim_config,
        std::make_unique<Autopilot>(autopilot_config),
        PhysicsModel{9.81, 0.01}
    };

    State initial;
    initial.x    =  50.0;
    initial.y    = 500.0;
    initial.vx   =  2.0;
    initial.vy   = -20.0;
    initial.mass =  500.0;

    SimulationEngine::Status result;

    // --- URUCHOMIENIE W ZALEŻNOŚCI OD WYBORU ---
    if (use_renderer)
    {
#ifdef ENABLE_RENDERING
        Renderer::Config renderer_config;
        renderer_config.width      = 1024;
        renderer_config.height     = 768;
        renderer_config.scale      = 1.0f;
        renderer_config.max_thrust = autopilot_config.max_thrust;

        Renderer renderer{renderer_config};
        SimulationEngine::Status last_status = SimulationEngine::Status::Running;

        // Callback dla trybu graficznego
        engine.set_step_callback([&](const State& s,
                                      const ThrustCommand& cmd,
                                      double time,
                                      SimulationEngine::Status status)
        {
            last_status = status;
            renderer.handle_events();
            renderer.draw(s, cmd, time, status);
        });

        result = engine.run(initial);

        // Utrzymaj okno otwarte po zakończeniu symulacji
        while (renderer.is_open())
        {
            renderer.handle_events();
            renderer.draw({}, {}, 0.0, last_status);
        }
#endif
    }
    else
    {
        double last_log = 0.0;
        
        // Callback dla trybu tekstowego
        engine.set_step_callback([&](const State& s,
                                      const ThrustCommand& cmd,
                                      double time,
                                      SimulationEngine::Status status)
        {
            // Wypisuj logi co 10 sekund symulacji lub gdy symulacja się zakończy
            if (time - last_log >= 10.0 || status != SimulationEngine::Status::Running)
            {
                const double thrust_pct = (cmd.fy / autopilot_config.max_thrust) * 100.0;
                std::cout << std::format(
                    "t={:6.1f}s  y={:7.1f}m  vy={:6.2f}m/s  x={:6.1f}m  vx={:5.2f}m/s  thrust={:5.1f}%\n",
                    time, s.y, s.vy, s.x, s.vx, thrust_pct);
                last_log = time;
            }
        });

        result = engine.run(initial);

        // Wypisz wynik końcowy
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
    }

    // Na koniec zawsze generujemy plik .csv, żeby mieć dostęp do pełnych danych
    engine.saveRaport("raport.csv");

    return 0;
}