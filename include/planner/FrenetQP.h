#ifndef FRENETQP_H
#define FRENETQP_H
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>
#include "FrenetDP.h"
#include "PathBoundary.h"
#include "common/FrenetState.h"
#include "common/PathQPPoint.h"

class FrenetQP {
public:
    std::vector<PathQPPoint> Solve(const std::vector<double>& l_ref, const std::vector<PathBoundaryPoint>& boundary,
                              const FrenetState& start_state);
};

#endif // FRENETQP_H