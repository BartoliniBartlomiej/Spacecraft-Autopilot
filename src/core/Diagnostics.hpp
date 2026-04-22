// src/core/Diagnostics.hpp
#pragma once

#include "control/PIDController.hpp"

struct ControlDiagnostics {
    double vertical_error    = 0.0;
    double horizontal_error  = 0.0;
    double vertical_output   = 0.0;
    double horizontal_output = 0.0;
    PIDController::Gains vertical_gains;
    PIDController::Gains horizontal_gains;
};