# Spacecraft Autopilot Simulation

A modular 2D spacecraft autopilot system written in modern C++20. The simulation models a spacecraft performing an autonomous vertical landing using a PID control strategy, with a physics engine, unit-tested core logic, and a real-time SFML visualization.

Built as a portfolio project demonstrating clean architecture, modern C++ practices, and practical control systems implementation.

![CI](https://github.com/BartoliniBartlomiej/Spacecraft-Autopilot/actions/workflows/ci.yml/badge.svg)

---

## Features

- **Physics engine** — 2D Newtonian motion with gravity, thrust, and atmospheric drag
- **RK4 integrator** — Runge-Kutta 4th order numerical integration
- **PID autopilot** — dual-axis controller for vertical descent and horizontal stabilization
- **Extensible control interface** — `ControlStrategy` abstraction allows plugging in LQR, MPC, or any custom algorithm
- **Unit tested** — GoogleTest coverage for physics, PID controller, and autopilot logic
- **CI pipeline** — GitHub Actions builds and runs all tests on every push
- **SFML visualization** — real-time rendering of trajectory, velocity, and HUD *(in progress)*

---

## Architecture

```
src/
├── core/
│   └── State.hpp               # spacecraft state: position, velocity, mass
├── physics/
│   ├── PhysicsModel.hpp/cpp    # force accumulation, drag, gravity
├── control/
│   ├── ControlStrategy.hpp     # abstract interface for control algorithms
│   ├── PIDController.hpp/cpp   # clamped PID with anti-windup
│   └── Autopilot.hpp/cpp       # dual-axis autopilot using PIDController
└── main.cpp

tests/
├── test_physics_model.cpp
├── test_pid_controller.cpp
└── test_autopilot.cpp
```

## UML
Relationships between main classes and interfaces in the simulation architecture

```mermaid
classDiagram
    %% Data Structures
    class State {
        +double x
        +double y
        +double vx
        +double vy
        +double mass
    }

    class ThrustCommand {
        +double fx
        +double fy
    }

    %% Interfaces and control classes
    class ControlStrategy {
        <<interface>>
        +compute(State, double dt)* ThrustCommand
        +reset()* void
    }

    class PIDController {
        -Gains m_gains
        -double m_output_min
        -double m_output_max
        -double m_integral
        -double m_prev_error
        +compute(double error, double dt) double
        +reset() void
    }

    class Autopilot {
        +Config m_config
        +compute(State, double dt) ThrustCommand
        +reset() void
    }

    %% Physics and Engine
    class PhysicsModel {
        -double m_gravity
        -double m_drag_coefficient
        +derivative(State, ThrustCommand) State
    }

    class SimulationEngine {
        -Config m_config
        -StepCallback m_callback
        +run(State) Status
        +step(State, double time) Status
        +set_step_callback(StepCallback) void
        +saveRaport() void
    }

    %% Visualization
    class Renderer {
        -Config m_config
        -sf::RenderWindow m_window
        +is_open() bool
        +handle_events() void
        +draw(State, ThrustCommand, double time, Status) void
    }

    %% Relationships
    ControlStrategy <|-- Autopilot : Inheritance (Autopilot implements ControlStrategy)
    Autopilot *-- "2" PIDController : Composition (Autopilot owns two PIDControllers)
    
    SimulationEngine o-- "1" ControlStrategy : Agregation (Engine uses a ControlStrategy)
    SimulationEngine *-- "1" PhysicsModel : Composition (Engine owns PhysicsModel)
    
    SimulationEngine ..> State : Uses
    SimulationEngine ..> ThrustCommand : Uses
    
    Autopilot ..> State : Depends on
    Autopilot ..> ThrustCommand : Constructs
    
    PhysicsModel ..> State : Depends on
    PhysicsModel ..> ThrustCommand : Depends on
    
    Renderer ..> State : Observes
    Renderer ..> ThrustCommand : Observes
```

---

## Build Instructions

### Requirements

- C++20 compiler (AppleClang 17+, GCC 13+, or Clang 15+)
- CMake 3.20+
- SFML 2.6 *(optional, required for visualization)*

### macOS

```bash
brew install cmake sfml
```

### Ubuntu

```bash
sudo apt-get install cmake build-essential libsfml-dev
```

### Build & Run

```bash
git clone https://github.com/BartoliniBartlomiej/Spacecraft-Autopilot.git
cd Spacecraft-Autopilot

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

./build/spacecraft
```

### Run Tests

```bash
ctest --test-dir build --output-on-failure
```

---

## How It Works

The simulation runs a fixed-timestep loop:

```
1. Read current State (position, velocity, mass)
2. Autopilot computes ThrustCommand via PID controllers
3. PhysicsModel accumulates forces (thrust + gravity + drag)
4. RK4 integrator advances State by dt
5. Log output
6. Repeat until landed or fuel exhausted
```

The vertical PID targets a safe descent velocity (default: 0 m/s), while the horizontal PID keeps the spacecraft aligned over the landing zone.

---

## PID Tuning

Default gains in `config/simulation.json`:

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| Vertical | 2.0 | 0.1 | 1.5 |
| Horizontal | 1.2 | 0.05 | 0.8 |

Adjust these values to experiment with different landing behaviours — higher `Kd` reduces overshoot, higher `Ki` corrects steady-state drift.

---

## Project Status

| Phase | Status |
|---|---|
| Core simulation (physics, integrator) | ✅ Done |
| PID controller | ✅ Done |
| Autopilot + ControlStrategy interface | ✅ Done |
| Unit tests (12/12 passing) | ✅ Done |
| CI pipeline | ✅ Done |
| SimulationEngine (main loop) | 🔄 In progress |
| SFML visualization | ⏳ Planned |
| JSON config loader | ⏳ Planned |
| CSV data export | ⏳ Planned |

---

## Possible Extensions

- Additional control strategies (LQR, bang-bang, MPC) via `ControlStrategy` interface
- ImGui overlay for live PID tuning
- Trajectory optimisation (minimum fuel landing)
- Multi-stage rocket with stage separation
- Data export for external plotting (matplotlib, MATLAB)

---
