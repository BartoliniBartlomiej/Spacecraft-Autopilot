cmake_minimum_required(VERSION 3.20)
project(SpacecraftAutopilot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(spacecraft
    src/main.cpp
    src/core/SimulationEngine.cpp
    src/physics/PhysicsModel.cpp
    src/control/PIDController.cpp
    src/control/Autopilot.cpp
)

target_include_directories(spacecraft PRIVATE src)

target_compile_options(spacecraft PRIVATE
    -Wall -Wextra -Wpedantic
)

enable_testing()
add_subdirectory(tests)