#include "planner/FrenetDP.h"

bool FrenetDP::ISCollision(const Node& node, const Obstacle& obs)
{
    double dist = std::sqrt(std::pow(node.x - obs.x,2) + std::pow(node.y - obs.y,2));
    double vehicle_radius = 1.5;
    return dist < (vehicle_radius + obs.radius);
}

double FrenetDP::DisToObstacle(const Node& node,const Obstacle& obs){
    double dist = std::sqrt(std::pow(node.x - obs.x,2) + std::pow(node.y - obs.y,2));
    double vehicle_radius = 1.5;
    return dist - (vehicle_radius + obs.radius);
}

std::vector<Node> FrenetDP::DP_Search(const std::vector<ReferencePoint>& reference_line, 
                                      const std::vector<PathBoundaryPoint>& boundary,
                                      const std::vector<Obstacle>& obstacles,double start_l)
{
    // Build Graph
    std::vector<std::vector<Node>> Graph;
    for(size_t i = 0; i < reference_line.size(); ++i)
    {
        std::vector<Node> row;

        double s = reference_line[i].s;
        double l_min = boundary[i].l_min;
        double l_max = boundary[i].l_max;

        for(double l = l_min; l <= l_max; l += 0.5)
        {
            double ref_x = reference_line[i].x;
            double ref_y = reference_line[i].y;
            double theta = reference_line[i].theta;

            double x = ref_x - l * std::sin(theta);
            double y = ref_y + l * std::cos(theta);

            row.emplace_back(s,l,x,y);
        }
        Graph.push_back(row);
    }

    // Start Node
    int start_index = -1;
    double min_diff = std::numeric_limits<double>::infinity();
    for(size_t j = 0; j < Graph[0].size(); ++j) {
        double diff = std::abs(Graph[0][j].l - start_l);
        if(diff < min_diff) {
            min_diff = diff;
            start_index = static_cast<int>(j);
        }
    }
    if(start_index == -1) {
        std::cout << "No valid start node!" << std::endl;
        return {};
    }

    Graph[0][start_index].cost = 0.0;

    if(Graph[0][start_index].is_obstacle) {
        std::cout << "Start node collision!" << std::endl;
        return {};
    }

    // Weights
    double w_smoothness = 1.0;
    double w_reference = 2.0;   // 增强参考线代价
    double w_collision = 10.0;
    double w_curvature = 1.0;
    double w_terminal = 20.0;   // 终点回归参考线权重

    // DP Search
    for(size_t i = 0; i < Graph.size() - 1; ++i) {
        for(size_t j = 0; j < Graph[i].size(); ++j) {
            Node& current = Graph[i][j];
            if(std::isinf(current.cost)) continue;

            for(size_t next_j = 0; next_j < Graph[i+1].size(); ++next_j) {
                Node& next = Graph[i+1][next_j];
                if(next.is_obstacle) continue;
                if(std::abs(next.l - current.l) > 1.0) continue;

                // Collision cost
                double collision_cost = 0.0;
                for(const auto& obs : obstacles) {
                    double dist = DisToObstacle(next, obs);
                    if(dist < 0.2) collision_cost += 1e6;
                    else if(dist < 3.0) collision_cost += 10.0/(dist * dist);
                    else if(dist < 5.0) collision_cost += 1.0/(dist * dist);
                }

                // Curvature cost
                double curvature_cost = 0.0;
                if(current.parent_i != -1 && current.parent_j != -1) {
                    Node& prev = Graph[current.parent_i][current.parent_j];
                    double ddl = next.l - 2.0 * current.l + prev.l;
                    if(std::abs(ddl) > 0.5) continue;
                    curvature_cost = ddl * ddl;
                }

                // Smoothness cost
                double smoothness_cost = std::pow(next.l - current.l,2);

                // Reference cost
                double ref_cost = std::pow(next.l,2);

                double transition_cost = 
                        w_smoothness * smoothness_cost + 
                        w_reference * ref_cost + 
                        w_collision * collision_cost + 
                        w_curvature * curvature_cost;

                double new_cost = current.cost + transition_cost;

                if(new_cost < next.cost) {
                    next.cost = new_cost;
                    next.parent_i = i;
                    next.parent_j = j;
                }
            }
        }
    }

    // Find Best Path with terminal cost
    int best_i = Graph.size() - 1;
    int best_j = -1;
    double min_cost = std::numeric_limits<double>::infinity();

    for(size_t j = 0; j < Graph.back().size(); ++j) {
        double terminal_cost = w_terminal * std::pow(Graph.back()[j].l, 2);
        double total_cost = Graph.back()[j].cost + terminal_cost;

        if(total_cost < min_cost) {
            min_cost = total_cost;
            best_j = j;
        }
    }

    if(best_j == -1) {
        std::cout << "No valid path!" << std::endl;
        return {};
    }

    // BackTrack
    std::vector<Node> path;
    int i = best_i;
    int j = best_j;
    while(i != -1 && j != -1) {
        Node& current = Graph[i][j];
        path.push_back(current);
        int pi = current.parent_i;
        int pj = current.parent_j;
        i = pi;
        j = pj;
    }
    std::reverse(path.begin(), path.end());

    return path;
}