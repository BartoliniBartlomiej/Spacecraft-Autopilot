// PhysicsModel.cpp

#include "physics/PhysicsModel.hpp"

PhysicsModel::PhysicsModel(double gravity, double drag_coefficient)
    : m_gravity{gravity},
    m_drag_coefficient{drag_coefficient}
{}

State PhysicsModel::derivative(const State& s, const ThrustCommand& cmd) const {
    State deriv;

    deriv.x = s.vx;
    deriv.y = s.vy;

    // drag force - proportional to velocity
    const double drag_x = -m_drag_coefficient * s.vx;
    const double drag_y = -m_drag_coefficient * s.vy;

    // F = ma -> a = F/m
    deriv.vx = (cmd.fx + drag_x) / s.mass;
    deriv.vy = (cmd.fy + drag_y - m_gravity * s.mass) / s.mass;

    deriv.mass = 0.0;

    return deriv;
}