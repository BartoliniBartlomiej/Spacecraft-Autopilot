// ConfigFile.hpp

#pragma once

#include <string>

class ConfigFile {
public:
    void saveConfigJSON(std::string appName, int version, bool isEnabled, double threshold, std::string author) const;

    void loadConfigJSON() const;

    /*
    vertical: 
    kp, ki, kd

    horizontal:
    kp, ki, kd
    
    State:
    mass, initial_x, initial_y, initial_vx, initial_vy
    */
};