// ControlStrategy.hpp

#pragma once

#include <optional>

#include "core/State.hpp"
#include "core/Diagnostics.hpp"
#include "physics/PhysicsModel.hpp"

class ControlStrategy {
public:
    virtual ~ControlStrategy() = default;

    [[nodiscard]] virtual ThrustCommand compute(const State& current_state, double dt) = 0;
    virtual void reset() = 0;

    [[nodiscard]] virtual std::optional<ControlDiagnostics> last_diagnostics() const { return std::nullopt; }

    
};