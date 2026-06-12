#ifndef STGRAPH_H
#define STGRAPH_H
#include <vector>
#include <string>
#include "common/ObstaclePredictor.h"

enum class STBoundaryType
{
    FOLLOW,
    YIELD,
    OVERTAKE,
    STOP
};

struct STPoint
{
    double s;
    double t;

    bool valid = true;
};

struct STBoundary
{
    int id;

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;

    STBoundaryType type;
};

class STGraph
{
public:
    STBoundary GenerateSTBoundary( const DynamicObstacle& obs,int obs_id);

    STBoundary GenerateFromPrediction(const std::vector<PredictedObstaclePoint>& prediction,const DynamicObstacle& obs,int obs_id);

    void SaveSTObstacles(const std::vector<STBoundary>& boundaries, const std::string& filename);

    double total_time_ = 10.0;
    double dt_ = 0.2;

    void SetReferenceLine(const std::vector<ReferencePoint>& ref_line);
    void SetTimeConfig(double total_time, double dt);

private:
    ObstaclePredictor predictor_;

    std::vector<ReferencePoint> ref_line_;

};


#endif // STGRAPH_H