// PhysicsModel.hpp

#pragma once 

#include "core/State.hpp"

struct ThrustCommand {
    double fx = 0.0; // vertical force [N]
    double fy = 0.0; // horizontal force [N]
};

class PhysicsModel {
private:
    double m_gravity;
    double m_drag_coefficient;

public:
    explicit PhysicsModel(double gravity = 9.81, double drag_coefficient = 0.01);

    [[nodiscard]] State derivative(const State& s, const ThrustCommand& cmd) const; // return derivative of state (dx/dt)
};