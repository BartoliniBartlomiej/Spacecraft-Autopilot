// Renderer.hpp

#pragma once

#include "core/State.hpp"
#include "physics/PhysicsModel.hpp"
#include "core/SimulationEngine.hpp"
#include <SFML/Graphics.hpp>
#include <vector>

class Renderer
{
public:
    struct Config
    {
        unsigned int width        = 1024;
        unsigned int height       = 768;
        float        scale        = 1.0f;   // piksele na metr
        float        ground_y     = 0.0f;   // wysokość ziemi w metrach
        double       max_thrust   = 15000.0;
    };

    explicit Renderer(Config config);

    // Zwraca false gdy użytkownik zamknie okno
    [[nodiscard]] bool is_open() const;

    void handle_events();

    void draw(const State& state,
              const ThrustCommand& cmd,
              double time,
              SimulationEngine::Status status);

private:
    [[nodiscard]] sf::Vector2f world_to_screen(float x, float y) const;

    Config                    m_config;
    sf::RenderWindow          m_window;
    sf::Font                  m_font;
    std::vector<sf::Vector2f> m_trail;
};