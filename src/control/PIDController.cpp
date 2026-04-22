#include "control/PIDController.hpp"
#include <algorithm>
#include <iostream>

PIDController::PIDController(Gains gains, double output_min, double output_max)
    : m_gains{gains}
    , m_output_min{output_min}
    , m_output_max{output_max}
{}

double PIDController::compute(double error, double dt) {
    // P
    const double p = m_gains.kp * error;

    // I
    m_integral += error * dt;
    const double i = m_gains.ki * m_integral;

    // D
    const double derivative = (dt > 0.0) ? (error - m_prev_error) / dt : 0.0;
    const double d =  m_gains.kd * derivative;

    m_prev_error = error;

    double output = std::clamp(p + i + d, m_output_min, m_output_max);

    return output;
}

void PIDController::reset() {
    m_integral = 0.0;
    m_prev_error = 0.0;
}