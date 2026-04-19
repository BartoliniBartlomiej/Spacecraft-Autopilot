// PIDConstroller.hpp

#pragma once

class PIDController
{
public:
    struct Gains
    {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
    };

    explicit PIDController(Gains gains, double output_min, double output_max);

    // compute regulator output, dt - step time [s]
    [[nodiscard]] double compute(double error, double dt);

    void reset();

private:
    Gains  m_gains;
    double m_output_min;
    double m_output_max;
    double m_integral    = 0.0;
    double m_prev_error  = 0.0;
};