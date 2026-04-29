#include "core/Spacecraft.hpp"
#include <numeric>

double Spacecraft::getTotalMaxThrust() const {
    return std::accumulate(thrusters.begin(), thrusters.end(), 0.0,
                           [](double sum, const Thruster& t) {
                               return sum + t.max_thrust;
                           });
}

void from_json(const nlohmann::json& j, Thruster& t) {
    j.at("name").get_to(t.name);
    j.at("max_thrust").get_to(t.max_thrust);
    j.at("pos_x").get_to(t.pos_x);
    j.at("pos_y").get_to(t.pos_y);
    j.at("dir_x").get_to(t.dir_x);
    j.at("dir_y").get_to(t.dir_y);
}

void to_json(nlohmann::json& j, const Thruster& t) {
    j = nlohmann::json{
        {"name", t.name},
        {"max_thrust", t.max_thrust},
        {"pos_x", t.pos_x},
        {"pos_y", t.pos_y},
        {"dir_x", t.dir_x},
        {"dir_y", t.dir_y}
    };
}

void from_json(const nlohmann::json& j, Spacecraft& s) {
    j.at("name").get_to(s.name);
    j.at("width").get_to(s.width);
    j.at("height").get_to(s.height);
    j.at("dry_mass").get_to(s.dry_mass);
    j.at("fuel_capacity").get_to(s.fuel_capacity);
    j.at("thrusters").get_to(s.thrusters);
}

void to_json(nlohmann::json& j, const Spacecraft& s) {
    j = nlohmann::json{
        {"name", s.name},
        {"width", s.width},
        {"height", s.height},
        {"dry_mass", s.dry_mass},
        {"fuel_capacity", s.fuel_capacity},
        {"thrusters", s.thrusters}
    };
}
