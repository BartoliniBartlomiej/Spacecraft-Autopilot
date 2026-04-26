// ConfigFile.cpp

#include "services/ConfigFile.hpp"

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

void ConfigFile::saveConfigJSON(std::string appName, int version, bool isEnabled, double threshold, std::string author) const {
    nlohmann::json config;
    config["application_name"] = appName;
    config["version"] = version;
    config["active"] = isEnabled;
    config["sensitivity_threshold"] = threshold;
    config["author_info"] = author;

    std::ofstream file("config.json");

    if (file.is_open()) {
        file << config.dump(4);
        file.close();
        std::cout << "Configuration saved to config.json" << std::endl;
    } else {
        std::cerr << "Error: Could not open file for writing!" << std::endl;
    }
}

void ConfigFile::loadConfigJSON() const {
    // Implementation for loading config from JSON
}