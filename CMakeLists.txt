cmake_minimum_required(VERSION 3.10)

project(osqp_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_executable(planner
    src/planner/FrenetDP.cpp
    src/planner/FrenetQP.cpp
    src/referenceLine/ReLi_Provider.cpp
    src/planner/PathBoundary.cpp
    src/referenceLine/CoordiTrans.cpp
    src/planner/STGraph.cpp
    src/planner/SpeedDP.cpp
    src/planner/DPSTCost.cpp
    src/planner/SpeedQP.cpp
    src/trajectory/TrajectoryGenerate.cpp
    src/predictor/ObstaclePredictor.cpp
    # src/planner/speeddptest.cpp
    # src/planner/DP_QP.cpp
    src/simulator/Simulator.cpp
    src/simulator/SimMain.cpp
    # src/trajectory/TrajectoryTest.cpp
)

# add_executable(stGraphTest
#     src/planner/STGraph.cpp
#     src/planner/STtest.cpp
# )

# add_executable(speedDptest
#     src/planner/STGraph.cpp
#     src/planner/SpeedDP.cpp
#     src/planner/speeddptest.cpp
#     src/planner/DPSTCost.cpp
#     src/planner/SpeedQP.cpp
# )

target_link_libraries(
    planner
    OsqpEigen::OsqpEigen
)


# target_link_libraries(
#     speedDptest
#     OsqpEigen::OsqpEigen
# )

target_include_directories(planner PRIVATE
    ${PROJECT_SOURCE_DIR}/include 
)

# target_include_directories(stGraphTest PRIVATE
#     ${PROJECT_SOURCE_DIR}/include 
# )

# target_include_directories(speedDptest PRIVATE
#     ${PROJECT_SOURCE_DIR}/include 
# )
