#include <gtest/gtest.h>
#include "physics/PhysicsModel.hpp"
#include "core/Spacecraft.hpp"

static Spacecraft createTestSpacecraft() {
    Spacecraft s;
    s.dry_mass = 50.0;
    s.fuel_capacity = 50.0;
    return s;
}

TEST(PhysicsModelTest, FreeFallAcceleration)
{
    Spacecraft sc = createTestSpacecraft();
    PhysicsModel physics{sc, 9.81, 0.0};

    State s;
    s.mass = 100.0;
    s.vy   = 0.0;

    ThrustCommand cmd;  // no thrust

    const State deriv = physics.derivative(s, cmd);

    EXPECT_NEAR(deriv.vy, -9.81, 1e-9);
    EXPECT_NEAR(deriv.vx, 0.0,   1e-9);
}

TEST(PhysicsModelTest, ThrustCounteractsGravity)
{
    Spacecraft sc = createTestSpacecraft();
    PhysicsModel physics{sc, 9.81, 0.0};

    State s;
    s.mass = 100.0;

    ThrustCommand cmd;
    cmd.fy = 9.81 * 100.0;  // F = mg

    const State deriv = physics.derivative(s, cmd);

    EXPECT_NEAR(deriv.vy, 0.0, 1e-9);
}

TEST(PhysicsModelTest, DragOpposeVelocity)
{
    Spacecraft sc = createTestSpacecraft();
    PhysicsModel physics{sc, 0.0, 1.0}; 

    State s;
    s.mass = 1.0;
    s.vx   = 10.0; 

    ThrustCommand cmd;

    const State deriv = physics.derivative(s, cmd);

    EXPECT_NEAR(deriv.vx, -10.0, 1e-9);
}
