// State.hpp

#pragma once

struct State 
{
    double x = 0.0; // Position in x-axis [m]
    double y = 0.0; // Position in y-axis [m]
    double vx = 0.0; // Velocity in x-axis [m/s]
    double vy = 0.0; // Velocity in y-axis [m/s]
    double mass = 0.0; // Mass of the spacecraft + fuel [kg]
    double dryMass = 300.0; // Dry mass of the spacecraft without fuel [kg]
};