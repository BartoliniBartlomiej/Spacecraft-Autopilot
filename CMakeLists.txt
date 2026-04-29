cmake_minimum_required(VERSION 3.20)
project(SpacecraftAutopilot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Try to find SFML in standard locations
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
)
FetchContent_MakeAvailable(json)

option(ENABLE_RENDERING "Enable SFML rendering" ON)

set(SOURCES
    src/main.cpp
    src/core/SimulationEngine.cpp
    src/core/Spacecraft.cpp
    src/physics/PhysicsModel.cpp
    src/control/PIDController.cpp
    src/control/Autopilot.cpp
    src/services/DataLogger.cpp
    src/multisimulation/BatchSimulation.cpp 
)

if(ENABLE_RENDERING)
    find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
    list(APPEND SOURCES src/rendering/Renderer.cpp)
endif()

add_executable(spacecraft ${SOURCES})

target_include_directories(spacecraft PRIVATE src)

target_compile_options(spacecraft PRIVATE
    -Wall -Wextra -Wpedantic
)

target_link_libraries(spacecraft PRIVATE
    nlohmann_json::nlohmann_json
)

if(ENABLE_RENDERING)
    target_link_libraries(spacecraft PRIVATE
        SFML::Graphics SFML::Window SFML::System)
    target_compile_definitions(spacecraft PRIVATE ENABLE_RENDERING)
endif()

enable_testing()
add_subdirectory(tests)