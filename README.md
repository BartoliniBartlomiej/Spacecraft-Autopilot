# Spacecraft Autopilot Simulation (C++)

## Project Overview

This project implements a **modular spacecraft autopilot system** in modern C++ (C++20). The system simulates a spacecraft operating in a 2D environment and applies real-time control algorithms to manage flight phases, with a primary focus on **autonomous vertical landing**.

The project emphasizes clean architecture, memory safety, unit-tested logic, and a working graphical visualization — demonstrating both engineering problem-solving and professional development practices.

---

## Objectives

- Design and implement a physics-based 2D simulation engine
- Develop an autopilot control system with a PID controller as the baseline strategy
- Apply SOLID principles and modular, extensible architecture
- Use modern C++ features (C++20)
- Ensure safe memory management via RAII and smart pointers
- Provide a real-time graphical visualization of the simulation (SFML)
- Cover core logic with unit tests (GoogleTest) — mandatory, not optional
- Automate builds and tests via GitHub Actions CI

---

## Core Features

### Flight Simulation

- 2D spacecraft motion with position, velocity, and orientation
- Physics model including:
  - Gravity (constant downward acceleration)
  - Thrust force (direction and magnitude controlled by autopilot)
  - Atmospheric drag (simplified linear model)
- Runge-Kutta 4th order (RK4) numerical integrator as default; Euler available for comparison

### Autopilot System

- Autonomous control of descent velocity and horizontal position
- Dual-axis PID controller (vertical + horizontal)
- Extensible `ControlStrategy` interface enabling future algorithms (LQR, MPC)
- Soft landing detection: touchdown triggers when vertical velocity is below a configurable threshold

### Graphical Visualization (SFML)

- Real-time rendering of spacecraft position and trajectory trail
- HUD overlay displaying: altitude, velocity, fuel remaining, engine throttle
- Visual ground plane with landing zone marker
- Configurable window resolution and frame rate

### Modular Architecture

- Clean separation of concerns across independent modules:
  - Simulation loop and timing
  - Physics and integration
  - Vehicle model (spacecraft, thruster, fuel)
  - Autopilot and control strategies
  - Rendering and HUD
  - Configuration loading and logging

---

## File Structure

```
spacecraft-autopilot/
│
├── CMakeLists.txt                  # Root build configuration
├── README.md                       # Project overview, build instructions, demo GIF
├── .github/
│   └── workflows/
│       └── ci.yml                  # GitHub Actions: build + test on push/PR
│
├── config/
│   └── simulation.json             # Default simulation parameters (mass, PID gains, etc.)
│
├── src/
│   ├── main.cpp                    # Entry point: initialise and run simulation
│   │
│   ├── core/
│   │   ├── SimulationEngine.hpp
│   │   ├── SimulationEngine.cpp    # Main loop, timing, module coordination
│   │   ├── State.hpp               # Plain data struct: position, velocity, mass, fuel
│   │   └── Integrator.hpp          # RK4 and Euler integrators (template, header-only)
│   │
│   ├── physics/
│   │   ├── PhysicsModel.hpp
│   │   └── PhysicsModel.cpp        # Force accumulation, drag, gravity, derivative fn
│   │
│   ├── control/
│   │   ├── ControlStrategy.hpp     # Abstract interface (pure virtual)
│   │   ├── Autopilot.hpp
│   │   ├── Autopilot.cpp           # Selects strategy, feeds state, returns commands
│   │   ├── PIDController.hpp
│   │   └── PIDController.cpp       # Generic clamped PID with anti-windup
│   │
│   ├── vehicle/
│   │   ├── Spacecraft.hpp
│   │   ├── Spacecraft.cpp          # Aggregates thruster + fuel tank, owns state
│   │   ├── Thruster.hpp
│   │   ├── Thruster.cpp            # Thrust vector, throttle clamping, gimbal angle
│   │   ├── FuelTank.hpp
│   │   └── FuelTank.cpp            # Fuel mass, consumption rate, empty detection
│   │
│   ├── rendering/
│   │   ├── Renderer.hpp
│   │   ├── Renderer.cpp            # SFML window, spacecraft sprite, trajectory trail
│   │   ├── HUD.hpp
│   │   └── HUD.cpp                 # Overlay: altitude, velocity, throttle, fuel bar
│   │
│   └── io/
│       ├── Logger.hpp
│       ├── Logger.cpp              # Console + optional CSV file logging
│       ├── ConfigLoader.hpp
│       └── ConfigLoader.cpp        # JSON config parsing (nlohmann/json)
│
└── tests/
    ├── CMakeLists.txt              # GoogleTest target
    ├── test_pid_controller.cpp     # Step response, overshoot, steady-state error
    ├── test_physics_model.cpp      # Free-fall, thrust forces, drag coefficient
    ├── test_integrator.cpp         # RK4 vs Euler accuracy comparison
    ├── test_fuel_tank.cpp          # Consumption, depletion, edge cases
    └── test_landing_scenario.cpp   # End-to-end: spacecraft reaches ground softly
```

---

## Technical Requirements

### Language and Tools

| Tool | Version | Role |
|---|---|---|
| C++ | C++20 | Core language standard |
| CMake | 3.20+ | Build system |
| SFML | 2.6 | Real-time 2D rendering |
| GoogleTest | 1.14 | Unit and integration tests |
| nlohmann/json | 3.11 | Configuration file parsing |
| GitHub Actions | — | CI: build + test pipeline |

---

### Memory Management

- All resources managed via RAII — no manual `new`/`delete`
- `std::unique_ptr` for exclusive ownership (e.g. `Spacecraft` owns `Thruster`)
- `std::shared_ptr` for resources referenced across modules (e.g. shared `State`)
- `std::weak_ptr` to break potential cyclic dependencies
- No owning raw pointers anywhere in the codebase

---

### Code Quality

- Clean Code principles: small focused classes, meaningful names, no magic numbers
- `const` correctness throughout — all read-only methods and parameters marked `const`
- Immutability preferred; `State` is a value type passed by value or `const&`
- `[[nodiscard]]` on all functions whose return value must not be silently discarded
- `constexpr` for compile-time constants (gravity, default timestep, etc.)
- No naked `else` after `return`; early returns preferred over deep nesting

### Modern C++ Features in Use

- `std::optional<T>` — nullable return values (e.g. landing detection result)
- `std::variant<...>` — type-safe simulation event representation
- `std::format` — all string formatting (no `printf`, no manual concatenation)
- `std::span` — non-owning views over contiguous data in integrators
- Concepts — constrain template parameters in `Integrator` (e.g. `Integratable<T>`)
- Structured bindings — unpack state derivatives cleanly in physics code

---

## Simulation Workflow

```
┌─────────────────────────────────────────────────┐
│                  Simulation Loop                │
│                                                 │
│  1. Read current State from Spacecraft          │
│  2. Autopilot computes ControlCommand           │
│  3. PhysicsModel accumulates forces             │
│  4. Integrator (RK4) advances State by dt       │
│  5. Spacecraft updates fuel and checks landing  │
│  6. Renderer draws frame + HUD                  │
│  7. Logger writes data row (optional)           │
│  8. Repeat until landed or fuel empty           │
└─────────────────────────────────────────────────┘
```

---

## Functional Requirements

- Simulate spacecraft motion in discrete time steps (configurable `dt`, default 10 ms)
- Achieve soft vertical landing: touchdown velocity ≤ 2 m/s
- Stabilise horizontal drift to within ±1 m of the target landing zone
- Load all tunable parameters from `config/simulation.json`:
  - Spacecraft dry mass and initial fuel mass
  - Maximum thrust force and specific impulse
  - PID gains (Kp, Ki, Kd) for vertical and horizontal axes
  - Initial position and velocity
  - Simulation time limit
- Log per-step data: timestamp, position, velocity, throttle, fuel remaining
- Detect and report landing outcome: success, crash (velocity too high), or fuel exhaustion

---

## Testing Requirements

Unit tests and scenario tests are **mandatory**. The CI pipeline must pass before any commit is considered complete.

### Unit Tests

- `PIDController` — verify proportional, integral, and derivative responses independently; test anti-windup clamp; confirm zero output at zero error
- `PhysicsModel` — free-fall matches analytical solution; thrust produces correct acceleration; drag is proportional to velocity squared
- `Integrator` — RK4 produces lower error than Euler for same step size on a known ODE
- `FuelTank` — consumption reduces mass correctly; `isEmpty()` triggers at zero; no negative mass

### Scenario Tests

- Soft landing from rest at 500 m altitude: autopilot lands within velocity and position tolerances
- Fuel exhaustion mid-flight: simulation terminates gracefully with correct status
- Disturbed initial condition (horizontal drift of 50 m): autopilot corrects and lands on target

### Coverage Target

Aim for ≥ 80% line coverage on `control/` and `physics/` modules.

---

## CI Pipeline (GitHub Actions)

```yaml
# .github/workflows/ci.yml — triggered on every push and pull request
jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install dependencies
        run: sudo apt-get install -y libsfml-dev cmake
      - name: Configure
        run: cmake -B build -DCMAKE_BUILD_TYPE=Release
      - name: Build
        run: cmake --build build --parallel
      - name: Run tests
        run: ctest --test-dir build --output-on-failure
```

The README badge reflects CI status at all times.

---

## Configuration File

`config/simulation.json` drives all tunable parameters. No hardcoded values in source.

```json
{
  "spacecraft": {
    "dry_mass_kg": 500.0,
    "initial_fuel_kg": 300.0,
    "initial_position": { "x": 0.0, "y": 500.0 },
    "initial_velocity": { "x": 5.0,  "y": -10.0 }
  },
  "thruster": {
    "max_thrust_n": 8000.0,
    "specific_impulse_s": 300.0,
    "max_gimbal_angle_deg": 15.0
  },
  "autopilot": {
    "vertical_pid":   { "kp": 2.0, "ki": 0.1, "kd": 1.5 },
    "horizontal_pid": { "kp": 1.2, "ki": 0.05, "kd": 0.8 }
  },
  "simulation": {
    "timestep_s": 0.01,
    "time_limit_s": 300.0,
    "landing_velocity_threshold_ms": 2.0,
    "log_to_file": true,
    "log_path": "output/simulation.csv"
  }
}
```

---

## Assumptions

- Simulation operates in 2D (x horizontal, y vertical); no rotation of spacecraft body
- Physics model is Newtonian — no relativistic effects, no atmospheric density variation
- Gravity is constant (9.81 m/s²); no orbital mechanics
- Real-time constraints are soft — simulation can run faster or slower than wall clock
- SFML visualization is a compile-time option; headless mode (no window) is supported for CI

---

## Non-Goals

- 3D simulation or rendering
- Orbital mechanics or multi-body physics
- Hardware-in-the-loop or real-time embedded deployment
- Machine learning-based control strategies
- Multiplayer or networked simulation

---

## Possible Extensions (post-MVP)

These are explicitly out of scope for the initial version but are architected for:

- Additional control strategies implementing `ControlStrategy`: LQR, bang-bang, MPC
- Trajectory optimisation (minimum fuel landing)
- Multi-stage rocket simulation with stage separation events
- Data export to CSV/JSON for external plotting (matplotlib, MATLAB)
- ImGui debug overlay for live PID tuning without restarting the simulation
- Scenario scripting: define disturbances and events in JSON

---

## Expected Outcome

A complete, buildable, tested GitHub repository demonstrating:

- Fluency with modern C++20 idioms and standard library
- Clean, layered architecture following SOLID principles
- Practical implementation of a PID control system in a physics simulation
- Professional development habits: unit tests, CI pipeline, structured configuration, clear README

The repository README must include a demo GIF or screenshot of the SFML visualization, build instructions for Linux and Windows, and a brief explanation of the PID tuning approach used.

---

## Development Priorities

Complete these in order. Do not move to the next phase until the current one is working and tested.

**Phase 1 — Core simulation (no rendering)**
Physics model, RK4 integrator, basic spacecraft state, console logging. Verified by unit tests.

**Phase 2 — Autopilot**
PID controller, Autopilot class, soft landing scenario. Verified by scenario test passing.

**Phase 3 — Visualization**
SFML renderer, trajectory trail, HUD. Manually verified against Phase 2 behaviour.

**Phase 4 — Polish**
JSON config loader, CSV logger, CI pipeline, README with demo GIF.

**Phase 5 — Extensions (optional)**
Any items from the extensions list above, only after Phase 4 is complete.