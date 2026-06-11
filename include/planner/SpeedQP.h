#ifndef SPEEDQP_H
#define SPEEDQP_H   

#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "planner/SpeedDP.h"

class SpeedQP
{
public:
    std::vector<double> QP_Optimize(const std::vector<SpeedNode>& path);

private:
    double dt_ = 0.2;
    double N_ = 0;
    double v_max = 16.0;
    double a_max = 4.0;
    double a_min = -4.0;
    double jerk_max = 8.0;
};

#endif