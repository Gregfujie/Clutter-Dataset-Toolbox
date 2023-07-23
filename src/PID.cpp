#include "PID.h"

PID::PID(double target, double upper, double lower, double dt){
    this->e = 0.0;
    this->pre_e = 0.0;
    this->dt = dt;
    this->target = target;
    this->upper_bound = upper;
    this->lower_bound = lower;
    this->kp = 1.0;
    this->ki = 0.0;
    this->kd = 0.0;
}

void PID::set_ideal(double target){
    this->target = target;
}

void PID::set_param(double kp, double ki, double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::set_bound(double upper, double lower){
    this->upper_bound = upper;
    this->lower_bound = lower;
}

double PID::calculate_output(double cur){
    this->e = this->target - cur;
    if (abs(this->e)<1e-4){return 0.0;}
    double v = 0.0;
    if(this->sum_e.size()>=2){
        double sum = 0.0;
        for(int i = 0; i< this->sum_e.size(); i++){
            sum += this->sum_e[i];
        }
        v = this->e * this->kp + sum * this->ki + (this->e - this->pre_e) / this->dt * this->kd;
    }
    else{
        v = this->e * this->kp;
    }

    if (v < this->lower_bound){
        v = this->lower_bound;
    }
    else if(v > this->upper_bound){
        v = this->upper_bound;
    }

    this->pre_e = this->e;
    this->sum_e.emplace_back(this->e);
    if(this->sum_e.size() > 10){
        this->sum_e.erase(begin(this->sum_e));
    }

    return v;
}