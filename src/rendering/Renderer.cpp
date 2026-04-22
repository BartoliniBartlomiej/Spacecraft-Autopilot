//  Renderer.cpp

#include "rendering/Renderer.hpp"
#include <format>
#include <iostream>

Renderer::Renderer(Config config)
    : m_config{config}
    , m_window{sf::VideoMode{{config.width, config.height}}, "Spacecraft Autopilot"} {
    m_window.setFramerateLimit(60);
    
    // std::cout << "Window created, isOpen: " << m_window.isOpen() << "\n";
    // std::cout << "Window size: " << m_window.getSize().x << "x" << m_window.getSize().y << "\n";
    
    if (!m_font.openFromFile("/System/Library/Fonts/Helvetica.ttc"))
    {
        std::cout << "Font failed to load\n";
    }
}

bool Renderer::is_open() const {
    return m_window.isOpen();
}

void Renderer::handle_events() {
    while (const std::optional event = m_window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
        {
            m_window.close();
        }
        // Obsługa klawiatury dla trybu przyspieszonego (Klawisz F)
        else if (const auto* keyEvent = event->getIf<sf::Event::KeyPressed>())
        {
            if (keyEvent->code == sf::Keyboard::Key::F)
            {
                m_fast_forward = !m_fast_forward;
                
                m_window.setFramerateLimit(m_fast_forward ? 0 : 60);
            }
        }
    }
}

sf::Vector2f Renderer::world_to_screen(float x, float y) const {

    const float screen_x = static_cast<float>(m_config.width)  / 2.0f + x * m_config.scale;
    const float screen_y = static_cast<float>(m_config.height) * 0.85f - y * m_config.scale;
    return {screen_x, screen_y};
}

void Renderer::draw(const State& state,
                    const ThrustCommand& cmd,
                    double time,
                    SimulationEngine::Status status) {
    m_window.clear(sf::Color{15, 15, 25});

    // Ground
    const float ground_screen_y = world_to_screen(0.0f, 0.0f).y;
    sf::RectangleShape ground{{static_cast<float>(m_config.width), 40.0f}};
    ground.setPosition({0.0f, ground_screen_y});
    ground.setFillColor(sf::Color{50, 120, 50});
    m_window.draw(ground);

    // Landing zone
    const sf::Vector2f landing_pos = world_to_screen(0.0f, 0.0f);
    sf::RectangleShape landing_zone{{60.0f, 6.0f}};
    landing_zone.setOrigin({30.0f, 3.0f});
    landing_zone.setPosition(landing_pos);
    landing_zone.setFillColor(sf::Color{220, 180, 0});
    m_window.draw(landing_zone);

    // Trajectory
    const sf::Vector2f pos = world_to_screen(
        static_cast<float>(state.x),
        static_cast<float>(state.y));

    m_trail.push_back(pos);

    for (size_t i = 1; i < m_trail.size(); ++i)
    {
        const auto alpha = static_cast<uint8_t>(255 * i / m_trail.size());
        sf::Vertex line[2];
        line[0] = sf::Vertex{m_trail[i - 1], sf::Color{100, 180, 255, alpha}};
        line[1] = sf::Vertex{m_trail[i],     sf::Color{100, 180, 255, alpha}};
        m_window.draw(line, 2, sf::PrimitiveType::Lines);
    }

    // Spacecraft — triangle
    sf::CircleShape spacecraft{10.0f, 3};
    spacecraft.setOrigin({10.0f, 10.0f});
    spacecraft.setPosition(pos);
    spacecraft.setFillColor(sf::Color{220, 220, 255});
    m_window.draw(spacecraft);

    // Thruster flame
    const double thrust_ratio = cmd.fy / m_config.max_thrust;
    if (thrust_ratio > 0.05)
    {
        const auto flame_height = static_cast<float>(thrust_ratio * 30.0f);
        sf::RectangleShape flame{{6.0f, flame_height}};
        flame.setOrigin({3.0f, 0.0f});
        flame.setPosition({pos.x, pos.y + 10.0f});
        flame.setFillColor(sf::Color{255, 140, 0, 200});
        m_window.draw(flame);
    }

    // HUD 
    sf::Text hud{m_font, "", 14};
    hud.setFillColor(sf::Color::White);
    hud.setPosition({10.0f, 10.0f});

    const double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    const double thrust_pct = (cmd.fy / m_config.max_thrust) * 100.0;
    
    hud.setString(std::format(
        "Time:    {:.1f} s\n"
        "Alt:     {:.1f} m\n"
        "Speed:   {:.2f} m/s\n"
        "Vy:      {:.2f} m/s\n"
        "Vx:      {:.2f} m/s\n"
        "Thrust:  {:.1f}%\n"
        "X drift: {:.1f} m\n"
        "Fast FWD: {}",
        time, state.y, speed, state.vy, state.vx, thrust_pct, state.x,
        (m_fast_forward ? "ON [Press F]" : "OFF [Press F]")));

    m_window.draw(hud);
    
    // Status message
    if (status == SimulationEngine::Status::Landed) {
        sf::Text result{m_font, "SOFT LANDING!", 32};
        result.setFillColor(sf::Color{0, 255, 100});
        result.setPosition({
            static_cast<float>(m_config.width)  / 2.0f - 100.0f,
            static_cast<float>(m_config.height) / 2.0f});
        m_window.draw(result);
    }
    else if (status == SimulationEngine::Status::Crashed) {
        sf::Text result{m_font, "CRASHED!", 32};
        result.setFillColor(sf::Color{255, 50, 50});
        result.setPosition({
            static_cast<float>(m_config.width)  / 2.0f - 80.0f,
            static_cast<float>(m_config.height) / 2.0f});
        m_window.draw(result);
    }

    m_window.display();
}