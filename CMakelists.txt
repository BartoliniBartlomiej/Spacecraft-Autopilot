cmake_minimum_required(VERSION 3.20)
project(SpacecraftAutopilot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Główny executable
add_executable(spacecraft src/main.cpp)

target_compile_options(spacecraft PRIVATE
    -Wall -Wextra -Wpedantic
)

# Testy
enable_testing()
add_subdirectory(tests)