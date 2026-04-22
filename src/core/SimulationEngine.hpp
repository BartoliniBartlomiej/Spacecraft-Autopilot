// SimulationEngine.hpp

#pragma once

#include "core/State.hpp"
#include "physics/PhysicsModel.hpp"
#include "control/ControlStrategy.hpp"
#include "control/Autopilot.hpp"

#include <iostream>
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <fstream>

inline void printLastLine(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::ate); // otwórz i od razu skocz na koniec (ate)

    if (!file.is_open()) {
        std::cerr << "Error: could not open file for reading: " << filePath << "\n";
        return;
    }

    // Pobierz pozycję kursora (rozmiar pliku)
    std::streampos size = file.tellg();
    if (size == 0) {
        std::cout << "File is empty.\n";
        return;
    }

    std::string lastLine = "";
    
    // Cofaj się od końca pliku (pomiń ostatni znak, jeśli to '\n')
    for (int i = 1; i <= size; ++i) {
        file.seekg(-i, std::ios::end);
        char ch;
        file.get(ch);

        // Jeśli znajdziesz znak nowej linii (i to nie jest sam koniec pliku)
        if (ch == '\n' && i > 1) {
            break;
        }
        
        if (ch != '\r' && ch != '\n') {
            lastLine = ch + lastLine;
        }
    }

    std::cout << "Last log: " << lastLine << std::endl;
    file.close();
}

class SimulationEngine {
public:
    struct Config {
        double timestep = 0.01; // [s]
        double time_limit = 100.0; // [s]
        double landing_altitude = 0.0; // [m]
        double max_landing_velocity = 2.0; // [m/s]
    };

    enum class Status {
        Running,
        Landed,
        Crashed,
        FuelExhausted,
        TimeLimitReached
    };

    // using StepCallback = std::function<void(const State&, double time, Status)>;
    using StepCallback = std::function<void(const State&, const ThrustCommand&, double time, Status)>;

    SimulationEngine(Config config,
                     std::unique_ptr<ControlStrategy> autopilot,
                     PhysicsModel physics);
    ~SimulationEngine(){
        saveReport("report.csv");
        std::cout   << "SimulationEngine destroyed. History size: " << m_history.size() << std::endl;
    };
    Status run(State initial_state); // run full simulation thru the end

    Status step(State& state, double& time); // single step - (for render???)

    void set_step_callback(StepCallback callback);
    void saveReport(const std::string& filename) const;
    
private:
    [[nodiscard]] Status check_status(const State& state) const;
    [[nodiscard]] State integrate(const State& state, const ThrustCommand& cmd) const;

    Config m_config;
    std::unique_ptr<ControlStrategy> m_autopilot;
    PhysicsModel m_physics;
    StepCallback m_callback;

    Status m_lastStatus = Status::Running;

    struct StepRecord {
        double time;
        State state;
        ThrustCommand cmd;
        Autopilot::Diagnostics diagnostics;
    };

    std::vector<StepRecord> m_history;

    std::string statusToString(SimulationEngine::Status status) const {
        switch (status) {
            case SimulationEngine::Status::Running:       return "Running";
            case SimulationEngine::Status::Landed:        return "Landed";
            case SimulationEngine::Status::Crashed:       return "Crashed";
            case SimulationEngine::Status::FuelExhausted: return "FuelExhausted";
            case SimulationEngine::Status::TimeLimitReached: return "TimeLimitReached";
            default:                                      return "Unknown";
        }
    }
};

