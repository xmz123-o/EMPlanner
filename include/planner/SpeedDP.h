#ifndef SPEEDDP_H
#define SPEEDDP_H

#include <vector>
#include <limits>
#include "planner/STGraph.h"

struct SpeedNode
{
    double s;
    double t;

    double cost;
    
    double speed_cost = 0.0;
    double obstacle_cost = 0.0;
    double accel_cost = 0.0;
    double jerk_cost = 0.0;

    int parent_i;
    int parent_j;

    SpeedNode(double s_,double t_) : s(s_),t(t_),cost(std::numeric_limits<double>::infinity()),parent_i(-1),parent_j(-1) {}
};

class SpeedDP
{
public:
    std::vector<SpeedNode> SpeedDP_Search(const std::vector<STBoundary>& boundaries,double init_v);
private:
    void SaveDPGrid(
    const std::vector<std::vector<SpeedNode>>& graph,
    const std::string& filename);
};



#endif // SPEEDDP_H