#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "simulator/EgoState.h"
#include "common/ReferencePoint.h"
#include "common/Obstacle.h"
#include "common/DynamicObstacle.h"
#include "common/TrajectoryPoint.h"
#include "planner/FrenetDP.h"
#include "planner/FrenetQP.h"
#include "planner/SpeedDP.h"
#include "planner/SpeedQP.h"
#include "planner/PathBoundary.h"
#include "trajectory/TrajectoryGenerate.h"
#include "common/CoordiTrans.h"

#include <vector>

struct DynamicObstacleHistory
{
    int frame;

    int id;

    double x;
    double y;
};

class Simulator
{
public:
    Simulator();

    void Init(const std::vector<ReferencePoint>& reference_line, 
              const std::vector<Obstacle>& static_obstacles, const std::vector<DynamicObstacle>& dynamic_obstacles);

    void Run();

private:
    std::vector<TrajectoryPoint> Plan();

    void Execute(const std::vector<TrajectoryPoint>& trajectory);

    void UpdateObstacle();

    bool ReachedGoal() const;

    ReferencePoint QueryReference(double s) const;

    std::vector<ReferencePoint> GetLocalRefLine(double start_s,double horizon_s);

    void SaveDynamicObstacleHistory();

    void SaveHistory();

    void SaveStaticObstacles();

private:
    double planning_dt_ = 0.2;

    EgoState ego_;

    std::vector<ReferencePoint> reference_line_;

    std::vector<Obstacle> static_obstacles_;
    std::vector<ObstacleFrenet> static_obstacles_frenet_;

    std::vector<DynamicObstacle> dynamic_obstacles_;

    std::vector<EgoState> ego_history_;

    PathBoundaryGenerator boundaryGen_;

    FrenetDP frenet_dp_;

    FrenetQP frenet_qp_;

    TrajectoryGenerate traj_generator_;

    STGraph st_graph_;

    SpeedDP speed_dp_;

    SpeedQP speed_qp_;

    std::vector<DynamicObstacleHistory> dynamic_obs_history_;
};

#endif