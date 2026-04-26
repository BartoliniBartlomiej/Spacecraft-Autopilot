#include <iostream>
#include <format>
#include <string>

#include "core/SimulationEngine.hpp"
#include "control/Autopilot.hpp"
#include "multisimulation/BatchSimulation.hpp"
#include "services/DataLogger.hpp"

#ifdef ENABLE_RENDERING
#include "rendering/Renderer.hpp"
#endif

// ---------------------------------------------------------------------------
// Shared simulation parameters
// ---------------------------------------------------------------------------

static Autopilot::Config makeAutopilotConfig()
{
    Autopilot::Config cfg;
    cfg.vertical_gains   = {.kp = 60.0, .ki = 0.0, .kd = 0.0};
    cfg.horizontal_gains = {.kp = 200.0,   .ki = 0.5, .kd = 150.0};
    cfg.max_thrust       = 15000.0;
    cfg.target_vy        = -1.5;   // target descent velocity [m/s]
    cfg.target_x         = 0.0;
    return cfg;
}

static SimulationEngine::Config makeSimConfig()
{
    SimulationEngine::Config cfg;
    cfg.timestep             = 0.01;
    cfg.time_limit           = 600.0;
    cfg.max_landing_velocity = 2.0;
    cfg.landing_altitude     = 0.0;
    return cfg;
}

static State makeInitialState()
{
    State s;
    s.x       =  50.0;
    s.y       = 500.0;
    s.vx      =  2.0;
    s.vy      = -50.0;
    s.mass    =  500.0;
    s.dryMass =  300.0;
    return s;
}

// ---------------------------------------------------------------------------
// Mode 1 — single simulation, no visualization (console output)
// ---------------------------------------------------------------------------

static void runHeadless()
{
    std::cout << "\n[MODE] Single simulation — headless\n";
    std::cout << std::string(50, '-') << "\n";

    SimulationEngine engine{
        makeSimConfig(),
        std::make_unique<Autopilot>(makeAutopilotConfig()),
        PhysicsModel{9.81, 0.01}
    };

    double last_log = 0.0;

    engine.set_step_callback(
        [&](const State& s, const ThrustCommand& cmd, double time, SimulationEngine::Status status)
        {
            if (time - last_log >= 5.0 || status != SimulationEngine::Status::Running)
            {
                const double thrust_pct = (cmd.fy / makeAutopilotConfig().max_thrust) * 100.0;
                std::cout << std::format(
                    "t={:6.1f}s  y={:7.1f}m  vy={:6.2f}m/s"
                    "  x={:6.1f}m  vx={:5.2f}m/s  thrust={:5.1f}%\n",
                    time, s.y, s.vy, s.x, s.vx, thrust_pct);
                last_log = time;
            }
        });

    const auto result = engine.run(makeInitialState());

    std::cout << std::string(50, '-') << "\n";
    switch (result)
    {
        case SimulationEngine::Status::Landed:
            std::cout << "✓ Soft landing successful!\n"; break;
        case SimulationEngine::Status::Crashed:
            std::cout << "✗ Crashed — touchdown velocity too high!\n"; break;
        case SimulationEngine::Status::FuelExhausted:
            std::cout << "✗ Fuel exhausted before landing!\n"; break;
        default:
            std::cout << "✗ Time limit reached!\n"; break;
    }

    DataLogger logger;
    logger.saveReport("output_data/single_headless.csv", engine);
}

// ---------------------------------------------------------------------------
// Mode 2 — single simulation with SFML visualization
// ---------------------------------------------------------------------------

static void runWithVisualization()
{
#ifdef ENABLE_RENDERING
    std::cout << "\n[MODE] Single simulation — SFML visualization\n";
    std::cout << std::string(50, '-') << "\n";
    std::cout << "Controls: [F] toggle fast-forward, [X] close window\n\n";

    const auto autopilot_cfg = makeAutopilotConfig();

    SimulationEngine engine{
        makeSimConfig(),
        std::make_unique<Autopilot>(autopilot_cfg),
        PhysicsModel{9.81, 0.01}
    };

    Renderer::Config renderer_cfg;
    renderer_cfg.width      = 1024;
    renderer_cfg.height     = 768;
    renderer_cfg.scale      = 1.0f;
    renderer_cfg.max_thrust = static_cast<double>(autopilot_cfg.max_thrust);

    Renderer renderer{renderer_cfg};
    SimulationEngine::Status last_status = SimulationEngine::Status::Running;

    engine.set_step_callback(
        [&](const State& s, const ThrustCommand& cmd, double time, SimulationEngine::Status status)
        {
            last_status = status;
            renderer.handle_events();
            if (renderer.is_open())
                renderer.draw(s, cmd, time, status);
        });

    const auto result = engine.run(makeInitialState());

    // Keep window open after simulation ends so the user can see the result
    while (renderer.is_open())
    {
        renderer.handle_events();
        renderer.draw({}, {}, 0.0, result);
    }

    DataLogger logger;
    logger.saveReport("output_data/single_visual.csv", engine);

#else
    std::cout << "\n[Warning] This binary was compiled without SFML support.\n";
    std::cout << "Rebuild with: cmake -B build -DENABLE_RENDERING=ON\n";
    std::cout << "Falling back to headless mode.\n";
    runHeadless();
#endif
}

// ---------------------------------------------------------------------------
// Mode 3 — batch / multi-simulation (PID grid search)
// ---------------------------------------------------------------------------

static void runBatch()
{
    std::cout << "\n[MODE] Multi-simulation — PID grid search\n";

    const auto base_cfg = makeAutopilotConfig();
    const auto sim_cfg  = makeSimConfig();
    const auto initial  = makeInitialState();

    auto scenarios = BatchSimulator::buildGridSearch(
        // vertical gains sweep
        {.kp_max = 100.0, .kp_step = 5.0,
        .ki_max =  10.0, .ki_step = 5.0,
        .kd_max =  50.0, .kd_step = 5.0},
        // horizontal gains sweep
        {.kp_max = 100.0, .kp_step = 5.0,
        .ki_max =  10.0, .ki_step = 5.0,
        .kd_max =  50.0, .kd_step = 5.0},
        initial, sim_cfg, base_cfg);

    std::cout << std::format("Grid generated: {} scenarios\n", scenarios.size());

    BatchSimulator::run(scenarios, /*save_reports=*/false);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║    Spacecraft Autopilot Simulator    ║\n";
    std::cout << "╚══════════════════════════════════════╝\n\n";

    std::cout << "  1. Single simulation (headless, console output)\n";
    std::cout << "  2. Single simulation (SFML visualization)\n";
    std::cout << "  3. Multi-simulation  (PID grid search, CSV export)\n";
    std::cout << "\nChoose mode [1/2/3]: ";

    int choice = 0;
    std::cin >> choice;

    switch (choice)
    {
        case 1: runHeadless();           break;
        case 2: runWithVisualization();  break;
        case 3: runBatch();              break;
        default:
            std::cout << "\nInvalid choice. Running headless by default.\n";
            runHeadless();
            break;
    }

    return 0;
}