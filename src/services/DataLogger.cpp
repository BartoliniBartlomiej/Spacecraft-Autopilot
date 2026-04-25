// DataLogger.cpp

#include "services/DataLogger.hpp"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <format>


void DataLogger::saveReport(const std::string& filename, SimulationEngine& engine, bool dontSaveReport) const {
    std::string finalPath = "-";

    if (dontSaveReport) {
        std::cout << "Report saving skipped (dontSaveReport=true).\n";
    } else {
        std::filesystem::path folder = "output_data";
        std::string baseName = std::filesystem::path(filename).stem().string();
        std::string extension = std::filesystem::path(filename).extension().string();

        std::filesystem::create_directories(folder);

        // std::string finalPath;
        int counter = 1;

        do {
            finalPath = (folder / (baseName + "_" + std::to_string(counter) + extension)).string();
            counter++;
        } while (std::filesystem::exists(finalPath));

        std::ofstream file(finalPath);

        if (!file.is_open()) {
            std::cerr << "Error: cannot open file: " << finalPath << "\n";
            return;
        }

        file << "time,X,Y,Vx,Vy,Mass,ThrustX,ThrustY,"
            << "VerticalError,HorizontalError,"
            << "VerticalOutput,HorizontalOutput,"
            << "\n";

        int i = 10;
        for (const auto& r : engine.getHistory()) {
            if (i % 10 == 0) { // log every 10th step to reduce file size
                file << std::format(
                "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
                "{:.4f},{:.4f},{:.4f},{:.4f},"
                "\n",
                r.time,
                r.state.x, r.state.y,
                r.state.vx, r.state.vy,
                r.state.mass,
                r.cmd.fx, r.cmd.fy,
                r.diagnostics.vertical_error,
                r.diagnostics.horizontal_error,
                r.diagnostics.vertical_output,
                r.diagnostics.horizontal_output);
            }
            i++;
        }

        std::cout << "Report saved to: " << finalPath << "\n";
    }

    // Multisimulation config's file
    std::string config_filepath = "output_data/configurations.csv";
    bool fileExists = std::filesystem::exists(config_filepath);

    std::ofstream config_file(config_filepath, std::ios::app); // append mode

    if (!config_file.is_open())
    {
        std::cerr << "Error: cannot open file: " << config_filepath << "\n";
        return;
    }

    if (!fileExists) {
        config_file << "filename,status,mass,initial_x,initial_y,initial_vx,initial_vy,Kp_v,Ki_v,Kd_v,Kp_h,Ki_h,Kd_h,Time\n";
    }

    if (!engine.getHistory().empty()) {
        const auto& r = engine.getHistory().front();
        config_file << std::format("{},{},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n",
            finalPath,
            engine.statusToString(engine.getStatus()),
            r.state.mass,
            r.state.x,
            r.state.y,
            r.state.vx,
            r.state.vy,
            r.diagnostics.vertical_gains.kp,
            r.diagnostics.vertical_gains.ki,
            r.diagnostics.vertical_gains.kd,
            r.diagnostics.horizontal_gains.kp,
            r.diagnostics.horizontal_gains.ki,
            r.diagnostics.horizontal_gains.kd,
            engine.getHistory().back().time);
    }

    config_file.close();
    std::cout << "Configuration logged to: " << config_filepath << "\n";
}