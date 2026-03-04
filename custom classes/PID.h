#ifndef PID_H
#define PID_H

#include <algorithm>

class PID {
private:
    double dt;
    double e_sum = 0.0;
    double e_prev = 0.0;
    double Kp = 0.0, Ki = 0.0, Kd = 0.0;
    double limit = 0.0;

public:
    PID(double sampleTime, double p, double i, double d, double sat = 0) 
        : dt(sampleTime), Kp(p), Ki(i), Kd(d), limit(sat) {}

    double compute(double setpoint, double current_value);

    void setGains(double Kp, double Ki, double Kd);

    void setLimit(double limit);

    void reset();
};

#endif