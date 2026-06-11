#include "planner/FrenetQP.h"
#include "common/PathQPPoint.h"

std::vector<PathQPPoint> FrenetQP::Solve(const std::vector<double>& l_ref, const std::vector<PathBoundaryPoint>& boundary,
                                    const FrenetState& start_state)
{
    const int N = l_ref.size(); // QP size

    //weight
    double w_ref = 2.0;
    double w_smooth = 5.0;
    double w_curvature = 3.0;

    //Hessian Matrix
    Eigen::SparseMatrix<OSQPFloat> H(N,N);
    std::vector<Eigen::Triplet<OSQPFloat>> H_triplets;

    //Gradient Vector
    Eigen::VectorXd g(N);
    g.setZero();

    //Reference Cost
    for(int i = 0; i < N; ++i) {
        H_triplets.emplace_back(i,i,2.0 * w_ref);
        g(i) = -2.0 * w_ref * l_ref[i];
    }

    //Smoothness Cost
    for(int i = 0; i < N-1; ++i) {
        H_triplets.emplace_back(i,i,2.0 * w_smooth);
        H_triplets.emplace_back(i+1,i+1,2.0 * w_smooth);
        H_triplets.emplace_back(i,i+1,-2.0 * w_smooth);
        H_triplets.emplace_back(i+1,i,-2.0 * w_smooth);
    }

    //Curvature Cost
    for(int i = 1; i < N-1; ++i)
    {
        H_triplets.emplace_back(i-1, i-1,  2.0 * w_curvature);

        H_triplets.emplace_back(i,   i,    8.0 * w_curvature);

        H_triplets.emplace_back(i+1, i+1,  2.0 * w_curvature);

        H_triplets.emplace_back(i-1, i,   -4.0 * w_curvature);

        H_triplets.emplace_back(i, i-1,   -4.0 * w_curvature);

        H_triplets.emplace_back(i, i+1,   -4.0 * w_curvature);

        H_triplets.emplace_back(i+1, i,   -4.0 * w_curvature);

        H_triplets.emplace_back(i-1, i+1, 2.0 * w_curvature);

        H_triplets.emplace_back(i+1, i-1, 2.0 * w_curvature);
    }

    const double eps_reg = 1e-6;
    for (int i = 0; i < N; ++i) {
        H_triplets.emplace_back(i, i, eps_reg);
    }

    //Assemble the Hessian matrix
    H.setFromTriplets(H_triplets.begin(), H_triplets.end());

    //Constraints A
    Eigen::SparseMatrix<OSQPFloat> A(N + 1,N);
    std::vector<Eigen::Triplet<OSQPFloat>> A_triplets;

    A_triplets.emplace_back(0,0,1.0);
    for(int i = 1; i < N + 1; ++i) {
        A_triplets.emplace_back(i,i - 1,1.0);
    }
    A.setFromTriplets(A_triplets.begin(),A_triplets.end());

    //Bounds
    Eigen::VectorXd l(N+1);
    Eigen::VectorXd u(N+1);
    l(0) = start_state.l;
    u(0) = start_state.l;

    for(int i = 0; i < N; ++i) {
        l(i+1) = boundary[i].l_min;
        u(i+1) = boundary[i].l_max;
    }

    //Setup OSQP Solver
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(true);
    solver.data()->setNumberOfVariables(N);
    solver.data()->setNumberOfConstraints(N+1);
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l); 
    solver.data()->setUpperBound(u);

    if(!solver.initSolver()) {
        std::cerr << "Failed to initialize OSQP solver!" << std::endl;
        return {};
    }

    solver.solveProblem();

    //Result
    Eigen::VectorXd l_opt = solver.getSolution();
    std::cout << "Optimal Lateral Offsets:\n" << l_opt.transpose() << std::endl;

    // Return the optimal lateral offsets
    std::vector<PathQPPoint> result;
    for(size_t i = 0; i < l_opt.size(); ++i) {
        PathQPPoint pt;

        pt.s = boundary[i].s;
        pt.l = l_opt(i);
        
        result.push_back(pt);
    }

    return result;
}