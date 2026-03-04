#include "PID.h"

PID::PID(double sampleTime, double p, double i, double d, double sat = 0) 
    : dt(sampleTime), Kp(p), Ki(i), Kd(d), limit(sat) {}

double PID::compute(double setpoint, double current_value) {
    double error = setpoint - current_value;
    
    // 1. Proportional
    double Pout = Kp * error;

    // 2. Integral (with basic anti-windup check)
    e_sum += error * dt;
    double Iout = Ki * e_sum;

    // 3. Derivative (on Measurement to avoid kicks)
    double derivative = (error - e_prev) / dt;
    double Dout = Kd * derivative;

    double output = Pout + Iout + Dout;

    // 4. Saturation
    if (limit > 0) {
        double clamped_output = std::clamp(output, -limit, limit);
        
        // Basic Anti-Windup: If saturated, stop integrating
        if (output != clamped_output) {
            e_sum -= error * dt; 
            output = clamped_output;
        }
    }

    e_prev = error;
    return output;
}

void PID::setGains(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::setLimit(double limit) {
    this->limit = limit;
}

void PID::reset() {
    this->e_sum = 0.0;
    this->e_prev = 0.0;
}