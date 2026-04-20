// ControlStrategy.hpp

#pragma once

#include "core/State.hpp"
#include "physics/PhysicsModel.hpp"

class ControlStrategy {
public:
    virtual ~ControlStrategy() = default;

    [[nodiscard]] virtual ThrustCommand compute(const State& current_state, double dt) = 0;
    virtual void reset() = 0;
};