#include "simulator/Simulator.h"
#include "referenceLine/ReLi_Provider.h"

int main()
{
    //----------------------------------
    // Reference Line
    //----------------------------------
    ReferenceLineProvider ref_provider;
    std::vector<ReferencePoint> reference_line = ref_provider.GenerateSineCurve();
    
    //----------------------------------
    // Static Obstacle
    //----------------------------------

    std::vector<Obstacle> static_obstacles;

    static_obstacles.emplace_back(100.0, 0.5,1.0);  
    static_obstacles.emplace_back(40.0, 1.0,1.0);

    //----------------------------------
    // Dynamic Obstacle
    //----------------------------------

    std::vector<DynamicObstacle> dynamic_obstacles;

    DynamicObstacle obs;
    obs.x = 60.0;
    obs.y = -2.5;
    obs.vx = 1.0;
    obs.vy = 0.2;
    obs.length = 4.0;
    obs.width = 2.0;

    DynamicObstacle obs1;
    obs1.x = 80.0;
    obs1.y = -4;
    obs1.vx = 1.0;
    obs1.vy = 0.1;
    obs1.length = 4.0;
    obs1.width = 2.0;

    dynamic_obstacles.push_back(obs);
    dynamic_obstacles.push_back(obs1);

    //----------------------------------
    // Simulator
    //----------------------------------

    Simulator sim;

    sim.Init(reference_line,static_obstacles,dynamic_obstacles);

    sim.Run();

    return 0;
}