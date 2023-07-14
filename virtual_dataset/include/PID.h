#include <vector>
#include <iostream>
using namespace std;
class PID
{
public:
    double kp, ki, kd;
    double e, pre_e, dt, target, upper_bound, lower_bound;
    vector<double> sum_e;
    PID(double target, double upper, double lower, double dt);

    void set_coeff(double kp, double ki, double kd);

    void set_ideal(double target);

    void set_param(double kp, double ki, double kd);

    void set_bound(double upper, double lower);

    double calculate_output(double cur);

};