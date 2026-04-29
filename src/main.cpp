#include <iostream>
#include <format>
#include <string>
#include <fstream>
#include <filesystem>
#include <nlohmann/json.hpp>

#include "core/SimulationEngine.hpp"
#include "core/Spacecraft.hpp"
#include "control/Autopilot.hpp"
#include "multisimulation/BatchSimulation.hpp"
#include "services/DataLogger.hpp"

#ifdef ENABLE_RENDERING
#include "rendering/Renderer.hpp"
#endif

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Configuration Loading
// ---------------------------------------------------------------------------

static std::string pickSpacecraftFile() {
    std::vector<std::string> files;
    std::string assetsPath = "assets";
    
    if (fs::exists(assetsPath) && fs::is_directory(assetsPath)) {
        for (const auto& entry : fs::directory_iterator(assetsPath)) {
            if (entry.path().extension() == ".json") {
                files.push_back(entry.path().string());
            }
        }
    }

    if (files.empty()) return "";

    std::cout << "\nAvailable spacecraft configurations:\n";
    for (size_t i = 0; i < files.size(); ++i) {
        std::cout << std::format("  {}. {}\n", i + 1, files[i]);
    }
    
    std::cout << "Select spacecraft [1-" << files.size() << "] (default 1): ";
    int choice = 1;
    if (!(std::cin >> choice) || choice < 1 || choice > static_cast<int>(files.size())) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        choice = 1;
    }
    
    return files[choice - 1];
}

static Spacecraft loadSpacecraft(const std::string& filename) {
    if (filename.empty()) {
        std::cout << "[Info] No config file found, using default spacecraft.\n";
    } else {
        std::ifstream file(filename);
        if (file.is_open()) {
            try {
                nlohmann::json j;
                file >> j;
                return j.get<Spacecraft>();
            } catch (const std::exception& e) {
                std::cerr << "[Error] Failed to parse " << filename << ": " << e.what() << "\n";
            }
        }
    }

    // Fallback default
    Spacecraft s;
    s.name = "Eagle (Default)";
    s.width = 4.0;
    s.height = 6.0;
    s.dry_mass = 300.0;
    s.fuel_capacity = 200.0;
    s.thrusters = {{"Main", 15000.0, 0.0, -3.0, 0.0, 1.0}};
    return s;
}

// ---------------------------------------------------------------------------
// Shared simulation parameters
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Shared simulation parameters
// ---------------------------------------------------------------------------

struct Scenario {
    std::string name;
    State initial_state;
    SimulationEngine::Config sim_config;
    Autopilot::Config autopilot_config;
};

static Scenario makeLandingScenario(const Spacecraft& sc) {
    Scenario s;
    s.name = "Landing";
    
    s.initial_state.x = 50.0;
    s.initial_state.y = 500.0;
    s.initial_state.vx = 2.0;
    s.initial_state.vy = -50.0;
    s.initial_state.mass = sc.dry_mass + sc.fuel_capacity;

    s.sim_config.timestep = 0.01;
    s.sim_config.time_limit = 600.0;
    s.sim_config.max_landing_velocity = 2.0;
    s.sim_config.landing_altitude = 0.0;

    s.autopilot_config.vertical_gains = {.kp = 60.0, .ki = 0.0, .kd = 0.0};
    s.autopilot_config.horizontal_gains = {.kp = 200.0, .ki = 0.5, .kd = 150.0};
    s.autopilot_config.target_vy = -1.5;
    s.autopilot_config.target_x = 0.0;
    
    return s;
}

static Scenario makeLaunchScenario(const Spacecraft& sc) {
    Scenario s;
    s.name = "Rocket Launch";
    
    s.initial_state.x = 0.0;
    s.initial_state.y = 0.1; // tiny bit above ground
    s.initial_state.vx = 0.0;
    s.initial_state.vy = 0.0;
    s.initial_state.mass = sc.dry_mass + sc.fuel_capacity;

    s.sim_config.timestep = 0.01;
    s.sim_config.time_limit = 300.0;
    s.sim_config.target_altitude = 1000.0;

    s.autopilot_config.vertical_gains = {.kp = 100.0, .ki = 0.1, .kd = 50.0};
    s.autopilot_config.horizontal_gains = {.kp = 200.0, .ki = 0.5, .kd = 150.0};
    s.autopilot_config.target_vy = 1000.0; // aim for 80m/s ascent
    s.autopilot_config.target_x = 0.0;
    
    return s;
}

// ---------------------------------------------------------------------------
// Mode 1 — single simulation, no visualization (console output)
// ---------------------------------------------------------------------------

static void runHeadless(const Spacecraft& spacecraft, const Scenario& scenario)
{
    std::cout << std::format("\n[MODE] Single simulation — headless ({}, {})\n", spacecraft.name, scenario.name);
    std::cout << std::string(50, '-') << "\n";

    SimulationEngine engine{
        spacecraft,
        scenario.sim_config,
        std::make_unique<Autopilot>(spacecraft, scenario.autopilot_config),
        PhysicsModel{spacecraft, 9.81, 0.01}
    };

    double last_log = 0.0;
    const double max_thrust = spacecraft.getTotalMaxThrust();

    engine.set_step_callback(
        [&](const State& s, const ThrustCommand& cmd, double time, SimulationEngine::Status status)
        {
            if (time - last_log >= 5.0 || status != SimulationEngine::Status::Running)
            {
                const double thrust_pct = (cmd.fy / max_thrust) * 100.0;
                std::cout << std::format(
                    "t={:6.1f}s  y={:7.1f}m  vy={:6.2f}m/s"
                    "  x={:6.1f}m  vx={:5.2f}m/s  thrust={:5.1f}%\n",
                    time, s.y, s.vy, s.x, s.vx, thrust_pct);
                last_log = time;
            }
        });

    const auto result = engine.run(scenario.initial_state);

    std::cout << std::string(50, '-') << "\n";
    switch (result)
    {
        case SimulationEngine::Status::Landed:
            std::cout << "✓ Soft landing successful!\n"; break;
        case SimulationEngine::Status::TargetReached:
            std::cout << "✓ Target altitude reached! Orbit insertion successful.\n"; break;
        case SimulationEngine::Status::Crashed:
            std::cout << "✗ Crashed!\n"; break;
        case SimulationEngine::Status::FuelExhausted:
            std::cout << "✗ Fuel exhausted!\n"; break;
        default:
            std::cout << "✗ Time limit reached!\n"; break;
    }

    DataLogger logger;
    logger.saveReport("output_data/single_headless.csv", engine);
}

// ---------------------------------------------------------------------------
// Mode 2 — single simulation with SFML visualization
// ---------------------------------------------------------------------------

static void runWithVisualization(const Spacecraft& spacecraft, const Scenario& scenario)
{
#ifdef ENABLE_RENDERING
    std::cout << std::format("\n[MODE] Single simulation — SFML visualization ({}, {})\n", spacecraft.name, scenario.name);
    std::cout << std::string(50, '-') << "\n";
    std::cout << "Controls: [F] toggle fast-forward, [X] close window\n\n";

    SimulationEngine engine{
        spacecraft,
        scenario.sim_config,
        std::make_unique<Autopilot>(spacecraft, scenario.autopilot_config),
        PhysicsModel{spacecraft, 9.81, 0.01}
    };

    Renderer::Config renderer_cfg;
    renderer_cfg.width      = 1024;
    renderer_cfg.height     = 768;
    renderer_cfg.scale      = 1.0f;

    Renderer renderer{spacecraft, renderer_cfg};
    SimulationEngine::Status last_status = SimulationEngine::Status::Running;

    engine.set_step_callback(
        [&](const State& s, const ThrustCommand& cmd, double time, SimulationEngine::Status status)
        {
            last_status = status;
            renderer.handle_events();
            if (renderer.is_open())
                renderer.draw(s, cmd, time, status);
        });

    const auto result = engine.run(scenario.initial_state);

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
    runHeadless(spacecraft, scenario);
#endif
}

// ---------------------------------------------------------------------------
// Mode 3 — batch / multi-simulation (PID grid search)
// ---------------------------------------------------------------------------

static void runBatch(const Spacecraft& spacecraft, const Scenario& scenario)
{
    std::cout << "\n[MODE] Multi-simulation — PID grid search\n";

    const auto sc_ptr   = std::make_shared<Spacecraft>(spacecraft);

    auto scenarios = BatchSimulator::buildGridSearch(
        sc_ptr,
        100.0, 100.0, 100.0,  // kp_max, ki_max, kd_max
        10.0,                 // step
        scenario.initial_state, scenario.sim_config, scenario.autopilot_config);

    std::cout << std::format("Grid generated: {} scenarios\n", scenarios.size());

    [[maybe_unused]] auto results = BatchSimulator::run(scenarios, /*save_reports=*/true);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║    Spacecraft Autopilot Simulator    ║\n";
    std::cout << "╚══════════════════════════════════════╝\n\n";

    std::string configFile = pickSpacecraftFile();
    Spacecraft spacecraft = loadSpacecraft(configFile);

    std::cout << "\nChoose scenario:\n";
    std::cout << "  1. Rocket Launch (Ascent to 1000m)\n";
    std::cout << "  2. Landing (Descent from 500m)\n";
    std::cout << "Select scenario [1/2] (default 1): ";
    
    int sChoice = 1;
    std::cin >> sChoice;
    
    Scenario scenario;
    if (sChoice == 2) {
        scenario = makeLandingScenario(spacecraft);
    } else {
        scenario = makeLaunchScenario(spacecraft);
    }

    std::cout << "\nChoose simulation mode:\n";
    std::cout << "  1. Single simulation (headless, console output)\n";
    std::cout << "  2. Single simulation (SFML visualization)\n";
    std::cout << "  3. Multi-simulation  (PID grid search, CSV export)\n";
    std::cout << "\nChoose mode [1/2/3]: ";

    int choice = 0;
    std::cin >> choice;

    switch (choice)
    {
        case 1: runHeadless(spacecraft, scenario);           break;
        case 2: runWithVisualization(spacecraft, scenario);  break;
        case 3: runBatch(spacecraft, scenario);              break;
        default:
            std::cout << "\nInvalid choice. Running headless by default.\n";
            runHeadless(spacecraft, scenario);
            break;
    }

    return 0;
}