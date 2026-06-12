#include "simulator/Simulator.h"

#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>

Simulator::Simulator() {}

void Simulator::Init(const std::vector<ReferencePoint>& reference_line, 
                    const std::vector<Obstacle>& static_obstacles, const std::vector<DynamicObstacle>& dynamic_obstacles) 
{
    reference_line_ = reference_line;
    static_obstacles_ = static_obstacles;
    dynamic_obstacles_ = dynamic_obstacles;

    static_obstacles_frenet_.clear();
    for(const auto& obs : static_obstacles_) {
        static_obstacles_frenet_.push_back(CoordinateTransform::ObstacleToFrenet(obs,reference_line));
    }

    st_graph_.SetReferenceLine(reference_line_);
    st_graph_.SetTimeConfig(10.0,planning_dt_);

    ego_.x = reference_line.front().x;
    ego_.y = reference_line.front().y;

    ego_.s = 0.0;
    ego_.l = 0.0;

    ego_.v = 5.0;
    ego_.a = 0.0;

    ego_history_.push_back(ego_);

    std::ofstream ref_file("../output/[SIM]reference_line.csv");

    ref_file << "x,y,s\n";
    for(const auto& pt : reference_line_)
    {
        ref_file
            << pt.x << ","
            << pt.y << ","
            << pt.s
            << "\n";
    }
    ref_file.close();
}

void Simulator::SaveStaticObstacles()
{
    std::ofstream file("../output/[SIM]static_obstacles.csv");

    file << "x,y,radius\n";

    for(const auto& obs : static_obstacles_)
    {
        file
            << obs.x << ","
            << obs.y << ","
            << obs.radius
            << "\n";
    }

    file.close();
}

void Simulator::SaveHistory()
{
    std::ofstream file( "../output/[SIM]ego_history.csv");

    file << "frame,x,y,s,l,v,a\n";

    for(size_t i=0;i<ego_history_.size();i++)
    {
        const auto& ego = ego_history_[i];

        file
            << i << ","
            << ego.x << ","
            << ego.y << ","
            << ego.s << ","
            << ego.l << ","
            << ego.v << ","
            << ego.a
            << "\n";
    }

    file.close();
}

void Simulator::SaveDynamicObstacleHistory()
{
    std::ofstream file(
        "../output/[SIM]dynamic_obstacles.csv");

    file << "frame,id,x,y\n";

    for(const auto& obs : dynamic_obs_history_)
    {
        file
            << obs.frame << ","
            << obs.id << ","
            << obs.x << ","
            << obs.y
            << "\n";
    }

    file.close();
}

std::vector<ReferencePoint> Simulator::GetLocalRefLine(double start_s,double horizon_s)
{
    std::vector<ReferencePoint> local_ref;

    double end_s = start_s + horizon_s;

    bool inserted_start = false;

    for(const auto& pt : reference_line_)
    {
        if(pt.s < start_s)
            continue;

        if(!inserted_start)
        {
            local_ref.push_back(QueryReference(start_s));
            inserted_start = true;
        }

        if(pt.s > end_s)
            break;

        if(std::abs(pt.s - start_s) < 1e-3)
            continue;

        local_ref.push_back(pt);
    }

    return local_ref;
}

void Simulator::Run() {
    int cycle = 0;
    const int max_cycle = 5000;

    while (!ReachedGoal())
    {
        if(cycle > max_cycle)
        {
            std::cout<<"Simulation Timeout"<<std::endl;
            break;
        }

        std::cout << "Cycle:" << cycle++ << " Ego s: " << ego_.s << " l: " << ego_.l << " v: " << ego_.v << std::endl;

        UpdateObstacle();

        auto trajectory = Plan();

        if(trajectory.empty())
        {
            std::cout << "Planning Failed!" << std::endl;
            return;
        }

        Execute(trajectory);

        ego_history_.push_back(ego_);

        for(size_t i=0;i<dynamic_obstacles_.size();i++)
        {
            DynamicObstacleHistory hist;

            hist.frame = cycle;

            hist.id = i;

            hist.x = dynamic_obstacles_[i].x;

            hist.y = dynamic_obstacles_[i].y;

            dynamic_obs_history_.push_back(hist);
        }
    }
    
    std::cout<<"Reach Goal"<<std::endl;
    SaveHistory();
    SaveDynamicObstacleHistory();   
    SaveStaticObstacles();
}

std::vector<TrajectoryPoint> Simulator::Plan() 
{
    std::vector<TrajectoryPoint> trajectory;

    //Local Ref
    double remain = reference_line_.back().s - ego_.s;
    double horizon = std::min(80.0,remain);
    auto local_ref = GetLocalRefLine(ego_.s,horizon);
    if(local_ref.empty()) {
        return trajectory;
    }

    if(local_ref.size() < 3) {
        return {};
    }

    //PathBoundary
    auto boundary = boundaryGen_.Generate(local_ref,static_obstacles_frenet_);

    ego_.l = std::clamp(ego_.l, boundary.front().l_min, boundary.front().l_max);
        
    //Frenet DP
    auto dp_path = frenet_dp_.DP_Search(local_ref,boundary,static_obstacles_,ego_.l);

    if(dp_path.empty()) {
        return trajectory;
    }

    //Build l_ref
    std::vector<double> l_ref;
    for(const auto& node : dp_path) {
        l_ref.push_back(node.l);
    }

    if(boundary.size() != l_ref.size())
    {
        std::cout
            << "Boundary size mismatch"
            << std::endl;

        return {};
    }

    //Frenet QP
    FrenetState start_state;
    start_state.l = ego_.l;
    start_state.dl = ego_.dl;
    start_state.ddl = ego_.ddl;

    std::cout<<"******ego_l="<<ego_.l<<std::endl;

    auto path_qp = frenet_qp_.Solve(l_ref,boundary,start_state);

    for(size_t i=0;i<path_qp.size()-1;i++)
    {
        double ds = path_qp[i+1].s - path_qp[i].s;

        path_qp[i].dl = (path_qp[i+1].l - path_qp[i].l) / ds;
    }

    for(size_t i=0;i<path_qp.size()-1;i++)
    {
        double ds = path_qp[i+1].s - path_qp[i].s;

        path_qp[i].ddl = (path_qp[i+1].dl - path_qp[i].dl) / ds;
    }

    path_qp.back().ddl = path_qp[path_qp.size()-2].ddl;

    path_qp.back().dl = path_qp[path_qp.size()-2].dl;

    if(path_qp.empty()) {
        return trajectory;
    }

    std::vector<STBoundary> st_boundaries;

    constexpr double kFrontRange = 100.0;
    constexpr double kBackRange  = 20.0;
    constexpr double kLatRange   = 4.5;

    int obs_id = 0;

    for(const auto& obs : dynamic_obstacles_)
    {
        auto frenet = CoordinateTransform::CartesianToFrenet(obs.x,obs.y,local_ref);

        double ds = frenet.s - ego_.s;
        double dl = frenet.l;

        if(ds < -kBackRange || ds > kFrontRange || std::abs(dl) > kLatRange) {
            continue;
        }
        
        auto boundary = st_graph_.GenerateSTBoundary(obs,obs_id++);

        if(boundary.lower_points.empty())
        {
            continue;
        }

        for(auto& p : boundary.lower_points)
        {
            p.s -= ego_.s;
        }

        for(auto& p : boundary.upper_points)
        {
            p.s -= ego_.s;
        }

        st_boundaries.push_back(boundary);
    }

    std::cout << "\n===== ST BOUNDARY =====" << std::endl;

    for(const auto& b : st_boundaries)
    {
        std::cout
            << "obs "
            << b.id
            << std::endl;

        std::cout
            << "first s = "
            << b.lower_points.front().s
            << std::endl;

        std::cout
            << "last s = "
            << b.lower_points.back().s
            << std::endl;
    }

    std::cout
        << "ego_s = "
        << ego_.s
        << std::endl;

    auto speed_dp = speed_dp_.SpeedDP_Search(st_boundaries,ego_.v);

    if(speed_dp.empty())
    {
        std::cout << "SpeedDP Empty" << std::endl;
        return {};
    }

    auto speed_qp = speed_qp_.QP_Optimize(speed_dp);

    std::vector<SpeedQPPoint> speed_path;

    for(size_t i = 0;i < speed_qp.size();i++) {
        SpeedQPPoint pt;
        pt.s = speed_qp[i];
        pt.t = i * planning_dt_;

        if(i == 0) {
            pt.v = 0.0;
            pt.a = 0.0;
        }
        else {
            pt.v = (speed_qp[i] - speed_qp[i - 1]) / planning_dt_;
        }
        if(i >= 2) {
            pt.a = (speed_qp[i] - 2.0 * speed_qp[i - 1] + speed_qp[i - 2]) / (planning_dt_ * planning_dt_);
        }
        else {
            pt.a = 0.0;
        }

        speed_path.push_back(pt);
    }
   
    trajectory = traj_generator_.TrajGenerate(path_qp, speed_path, local_ref);

    return trajectory;
}

void Simulator::Execute(const std::vector<TrajectoryPoint>& trajectory) {
    if(trajectory.size() < 2)
    return;

    const auto& next = trajectory[1];

    double dt = planning_dt_;

    ego_.s = next.s;
    ego_.l = next.l;

    ego_.x = next.x;
    ego_.y = next.y;

    ego_.v = next.v;
    ego_.a = next.a;

    ego_.dl  = next.dl;
    ego_.ddl = next.ddl;

}

void Simulator::UpdateObstacle() 
{
    for(auto& obs : dynamic_obstacles_)
    {
        obs.x += obs.vx * planning_dt_;
        obs.y += obs.vy * planning_dt_;
    }
}

bool Simulator::ReachedGoal() const
{
    double goal_s = reference_line_.back().s;
    return ego_.s >= goal_s - 2.0;
}

ReferencePoint Simulator::QueryReference(double s) const
{
    if(s <= reference_line_.front().s)
    return reference_line_.front();

    if(s >= reference_line_.back().s)
    return reference_line_.back();

    for(size_t i=0;i<reference_line_.size()-1;i++)
    {
        const auto& p0 = reference_line_[i];
        const auto& p1 = reference_line_[i+1];

        if(s >= p0.s && s <= p1.s)
        {
            double ratio = (s-p0.s) / (p1.s-p0.s);

            ReferencePoint pt;

            pt.x = p0.x + ratio * (p1.x - p0.x);

            pt.y = p0.y + ratio * (p1.y - p0.y);

            pt.s = s;

            pt.theta = p0.theta + ratio * (p1.theta - p0.theta);

            return pt;
        }
    }

    return reference_line_.back();

}