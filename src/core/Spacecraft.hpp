#pragma once

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

struct Thruster {
    std::string name;
    double max_thrust; // [N]
    double pos_x;      // [m] relative to CoM
    double pos_y;      // [m] relative to CoM
    double dir_x;      // unit vector x
    double dir_y;      // unit vector y
};

class Spacecraft {
public:
    std::string name;
    double width;       // [m]
    double height;      // [m]
    double dry_mass;    // [kg]
    double fuel_capacity; // [kg]
    std::vector<Thruster> thrusters;

    [[nodiscard]] double getTotalMaxThrust() const;

    // JSON serialization
    friend void from_json(const nlohmann::json& j, Spacecraft& s);
    friend void to_json(nlohmann::json& j, const Spacecraft& s);
};


void from_json(const nlohmann::json& j, Thruster& t);
void to_json(nlohmann::json& j, const Thruster& t);
