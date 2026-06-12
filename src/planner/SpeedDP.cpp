#include "planner/SpeedDP.h"
#include "planner/STGraph.h"
#include "planner/DPSTCost.h"

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>
#include <fstream>



std::vector<SpeedNode> SpeedDP::SpeedDP_Search(const std::vector<STBoundary>& boundaries,double init_v)
{
    // graph config
    double total_time = 10.0;
    double dt = 0.2;

    double max_s = 100.0;
    double ds = 0.2;

    int time_size = static_cast<int>(total_time / dt) + 1;

    int s_size = static_cast<int>(max_s / ds) + 1;

    // build graph
    std::vector<std::vector<SpeedNode>> graph;

    for(int i = 0; i < time_size; ++i)
    {
        std::vector<SpeedNode> row;

        double t = i * dt;

        for(int j = 0; j < s_size; ++j)
        {
            double s = j * ds;

            row.emplace_back(s, t);
        }

        graph.push_back(row);
    }

    // start
    int start_j = 0;
    graph[0][start_j].cost = 0.0;

    double init_speed = init_v;
    double ref_speed = 10.0;
    double max_speed = 16.0;
    

    DpStCost dp_cost(boundaries);

    // DP search
    for(int i = 0; i < time_size - 1; ++i)
    {
        for(int j = 0; j < s_size; ++j)
        {
            SpeedNode& current = graph[i][j];

            if(std::isinf(current.cost))
            {
                continue;
            }

            for(int next_j = j; next_j < s_size; ++next_j)
            {
                SpeedNode& next = graph[i + 1][next_j];

                double delta_s = next.s - current.s;

                double delta_t = next.t - current.t;

                double v = delta_s / delta_t;

                // speed limit
                if(v > max_speed)
                {
                    continue;
                }
                //hard constraint
                if (dp_cost.IsCollision(current.s,current.t,next.s, next.t))
                {
                    continue;
                }

                // speed tracking cost
                double speed_cost = dp_cost.GetSpeedCost(v, ref_speed);
                double obstacle_cost = dp_cost.GetObstacleCost(next.s,next.t);

                double acc_cost = 0.0;
                int p_i = current.parent_i;
                int p_j = current.parent_j;
                if(p_i == -1) {
                    double acc = (v - init_speed) / delta_t;
                    acc_cost = 3.0 * (acc / 4.0) * (acc / 4.0);
                }
                if(p_i != -1 && p_j != -1) {
                    SpeedNode& parent = graph[p_i][p_j];
                    acc_cost = dp_cost.GetAccelCost(parent, current, next);
                }

                double jerk_cost = 0.0;
                if(p_i != -1) {
                    SpeedNode&  parent = graph[p_i][p_j];
                    int gp_i = parent.parent_i;
                    int gp_j = parent.parent_j;

                    if(gp_i != -1 && gp_j != -1) {
                        SpeedNode& grand_parent = graph[gp_i][gp_j];
                        jerk_cost = dp_cost.GetJerkCost(grand_parent, parent, current, next);
                    }
                }
                
                double transition_cost = speed_cost + obstacle_cost + acc_cost + jerk_cost;

                double new_cost = current.cost + transition_cost;

                if(new_cost < next.cost)
                {
                    next.cost = new_cost;

                    next.speed_cost = speed_cost;

                    next.obstacle_cost = obstacle_cost;

                    next.accel_cost = acc_cost;

                    next.jerk_cost = jerk_cost;

                    next.parent_i = i;
                    next.parent_j = j;
                }

            }
        }
    }

    // find best
    double min_cost = std::numeric_limits<double>::infinity();

    int best_i = -1;
    int best_j = -1;

    for(int j = 0; j < s_size; ++j)
    {
        if(graph.back()[j].cost < min_cost)
        {
            min_cost = graph.back()[j].cost;

            best_i = time_size - 1;

            best_j = j;
        }
    }

   this->SaveDPGrid(
    graph,
    "../output/dp_grid_cost.csv");

    // backtrack
    std::vector<SpeedNode> path;

    int i = best_i;
    int j = best_j;

    while(i != -1 && j != -1)
    {
        path.push_back(graph[i][j]);

        int pi = graph[i][j].parent_i;

        int pj = graph[i][j].parent_j;

        i = pi;
        j = pj;
    }

    if(best_i == -1 || best_j == -1)
    {
        std::cout
            << "SpeedDP failed: no reachable node"
            << std::endl;

        return {};
    }
    
    std::reverse(path.begin(), path.end());

    std::cout
    << "\n===== DP SPEED RESULT ====="
    << std::endl;

    for(size_t i=0;i<std::min((size_t)20,path.size());i++)
    {
        double v = 0.0;

        if(i > 0)
        {
            v =
            (path[i].s-path[i-1].s)/
            (path[i].t-path[i-1].t);
        }

        std::cout
            << path[i].t
            << " "
            << path[i].s
            << " "
            << v
            << " "
            << path[i].cost
            << std::endl;
    }

    return path;
}

void SpeedDP::SaveDPGrid(
    const std::vector<std::vector<SpeedNode>>& graph,
    const std::string& filename)
{
    std::ofstream file(filename);

    file << "t,s,total_cost,speed_cost,obs_cost,accel_cost,jerk_cost\n";

    for(const auto& row : graph)
    {
        for(const auto& node : row)
        {
            if(std::isinf(node.cost))
            {
                continue;
            }

            file
                << node.t << ","
                << node.s << ","
                << node.cost << ","
                << node.speed_cost << ","
                << node.obstacle_cost << ","
                << node.accel_cost << ","
                << node.jerk_cost
                << "\n";
        }
    }

    file.close();
}