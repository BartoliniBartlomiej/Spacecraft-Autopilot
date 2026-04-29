#include <gtest/gtest.h>
#include "core/Spacecraft.hpp"
#include <nlohmann/json.hpp>

TEST(SpacecraftTest, JsonSerialization) {
    std::string json_str = R"({
        "name": "Test Lander",
        "width": 5.0,
        "height": 8.0,
        "dry_mass": 400.0,
        "fuel_capacity": 300.0,
        "thrusters": [
            {
                "name": "Main",
                "max_thrust": 20000.0,
                "pos_x": 0.0,
                "pos_y": -4.0,
                "dir_x": 0.0,
                "dir_y": 1.0
            }
        ]
    })";

    auto j = nlohmann::json::parse(json_str);
    Spacecraft s = j.get<Spacecraft>();

    EXPECT_EQ(s.name, "Test Lander");
    EXPECT_DOUBLE_EQ(s.width, 5.0);
    EXPECT_DOUBLE_EQ(s.height, 8.0);
    EXPECT_DOUBLE_EQ(s.dry_mass, 400.0);
    EXPECT_DOUBLE_EQ(s.fuel_capacity, 300.0);
    ASSERT_EQ(s.thrusters.size(), 1);
    EXPECT_EQ(s.thrusters[0].name, "Main");
    EXPECT_DOUBLE_EQ(s.thrusters[0].max_thrust, 20000.0);
}

TEST(SpacecraftTest, TotalMaxThrust) {
    Spacecraft s;
    s.thrusters = {
        {"T1", 1000.0, 0, 0, 0, 0},
        {"T2", 2000.0, 0, 0, 0, 0}
    };
    EXPECT_DOUBLE_EQ(s.getTotalMaxThrust(), 3000.0);
}
