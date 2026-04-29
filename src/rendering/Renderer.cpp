//  Renderer.cpp

#include "rendering/Renderer.hpp"
#include <format>
#include <iostream>

Renderer::Renderer(const Spacecraft& spacecraft, Config config)
    : m_spacecraft{spacecraft}
    , m_config{config}
    , m_window{sf::VideoMode{{config.width, config.height}}, "Spacecraft Autopilot"} {
    m_window.setFramerateLimit(60);
    
    {
    if (!m_font.openFromFile("/System/Library/Fonts/Helvetica.ttc"))
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

    // Spacecraft Body
    const float w = static_cast<float>(m_spacecraft.width * m_config.scale);
    const float h = static_cast<float>(m_spacecraft.height * m_config.scale);
    
    sf::RectangleShape body{{w, h}};
    body.setOrigin({w / 2.0f, h / 2.0f});
    body.setPosition(pos);
    body.setFillColor(sf::Color{180, 180, 180});
    body.setOutlineThickness(1.0f);
    body.setOutlineColor(sf::Color::White);
    m_window.draw(body);

    // Nose cone (triangle on top)
    sf::ConvexShape nose;
    nose.setPointCount(3);
    nose.setPoint(0, {0.0f, -h/2.0f});
    nose.setPoint(1, {-w/2.0f, h/2.0f - h}); // Wait, relative to position?
    // Actually simpler:
    nose.setPoint(0, {pos.x, pos.y - h/2.0f - w/2.0f});
    nose.setPoint(1, {pos.x - w/2.0f, pos.y - h/2.0f});
    nose.setPoint(2, {pos.x + w/2.0f, pos.y - h/2.0f});
    nose.setFillColor(sf::Color{200, 50, 50});
    m_window.draw(nose);

    const double max_thrust = m_spacecraft.getTotalMaxThrust();

    // Thrusters and flames
    for (const auto& t : m_spacecraft.thrusters)
    {
        const float nozzle_w = std::max(2.0f, w * 0.1f);
        const float nozzle_h = std::max(3.0f, h * 0.05f);
        
        sf::RectangleShape nozzle{{nozzle_w, nozzle_h}};
        nozzle.setOrigin({nozzle_w / 2.0f, 0.0f});
        
        // Position relative to center (invert Y because SFML is Y-down)
        sf::Vector2f t_pos = pos + sf::Vector2f(
            static_cast<float>(t.pos_x * m_config.scale),
            static_cast<float>(-t.pos_y * m_config.scale)
        );
        
        nozzle.setPosition(t_pos);
        nozzle.setFillColor(sf::Color{80, 80, 80});
        m_window.draw(nozzle);

        // Flame for this thruster
        // Since we don't have individual control yet, we distribute total thrust
        const double thrust_ratio = cmd.fy / max_thrust;
        if (thrust_ratio > 0.05)
        {
            const auto flame_height = static_cast<float>(thrust_ratio * 50.0f);
            sf::RectangleShape flame{{nozzle_w * 0.8f, flame_height}};
            flame.setOrigin({nozzle_w * 0.4f, 0.0f});
            flame.setPosition(t_pos + sf::Vector2f(0.0f, nozzle_h));
            flame.setFillColor(sf::Color{255, 140, 0, 200});
            m_window.draw(flame);
        }
    }

    // HUD 
    sf::Text hud{m_font, "", 14};
    hud.setFillColor(sf::Color::White);
    hud.setPosition({10.0f, 10.0f});

    const double speed = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    const double thrust_pct = (cmd.fy / max_thrust) * 100.0;
    
    hud.setString(std::format(
        "Spacecraft: {}\n"
        "Time:    {:.1f} s\n"
        "Alt:     {:.1f} m\n"
        "Speed:   {:.2f} m/s\n"
        "Mass:   {:.2f} kg\n"
        "Vy:      {:.2f} m/s\n"
        "Vx:      {:.2f} m/s\n"
        "Thrust:  {:.1f}%\n"
        "Thrust N:  {:.2f}\n"
        "X drift: {:.1f} m\n"
        "Fast FWD: {}",
        m_spacecraft.name,
        time, state.y, speed, state.mass, state.vy, state.vx, cmd.fy, thrust_pct, state.x,
        (m_fast_forward ? "ON [Press F]" : "OFF [Press F]")));

    m_window.draw(hud);
    
    sf::Text hudLeft{m_font, "", 14};
    hudLeft.setFillColor(sf::Color::White);
    hudLeft.setPosition({800.0f, 10.0f});
    
    std::vector<std::string> thruster_info;
    for( const auto& t : m_spacecraft.thrusters) {
        thruster_info.push_back(std::format(
            "Thruster: {}\n"
            "  Max Thrust: {:.2f} N\n"
            "  Position: ({:.2f}, {:.2f}) m\n"
            "  Direction: ({:.2f}, {:.2f})\n\n",
            t.name, t.max_thrust, t.pos_x, t.pos_y, t.dir_x, t.dir_y
        ));
    }

    hudLeft.setString(std::format("Thrusters:\n{}", std::accumulate(thruster_info.begin(), thruster_info.end(), std::string(""))));

    m_window.draw(hudLeft);
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
    } else if (status == SimulationEngine::Status::FuelExhausted) {
        sf::Text result{m_font, "FUEL EXHAUSTED!", 32};
        result.setFillColor(sf::Color{255, 200, 50});
        result.setPosition({
            static_cast<float>(m_config.width)  / 2.0f - 120.0f,
            static_cast<float>(m_config.height) / 2.0f});
        m_window.draw(result);
    } else if (status == SimulationEngine::Status::TargetReached) {
        sf::Text result{m_font, "ALTITUDE REACHED!", 32};
        result.setFillColor(sf::Color{0, 255, 100});
        result.setPosition({
            static_cast<float>(m_config.width)  / 2.0f - 120.0f,
            static_cast<float>(m_config.height) / 2.0f});
        m_window.draw(result);
    }

    m_window.display();
}
