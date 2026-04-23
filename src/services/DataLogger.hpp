// DataLogger.hpp

#pragma once

#include <string>
#include <iostream>
#include <fstream>

#include "core/SimulationEngine.hpp"

class DataLogger {
public:
    void saveReport(const std::string& filename, SimulationEngine& engine, bool dontSaveReport = false) const;
};