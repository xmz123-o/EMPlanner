#include "planner/SpeedQP.h"
#include <iostream>
#include <fstream>

std::vector<double> SpeedQP::QP_Optimize(const std::vector<SpeedNode>& path) {
   
    N_ = path.size();

    std::vector<double> s_ref(N_);
    for(int i = 0; i < N_; ++i) {
        s_ref[i] = path[i].s;
    }

    for(const auto& node : s_ref) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    std::cout << "********************"<<std::endl;

    int n_vars = N_;
    int n_constraints = 4 * N_ - 6;

    double w_ref = 2.0;
    double w_acc = 5.0;
    double w_jerk = 10.0;

    //Hessian Matrix
    Eigen::SparseMatrix<OSQPFloat> H(n_vars,n_vars);
    std::vector<Eigen::Triplet<OSQPFloat>> H_triplets;

    Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);

    //reference cost
    for(int i = 0; i < N_; ++i) {
        H_triplets.emplace_back(i,i,2.0 * w_ref);
        g[i] = -2.0 * w_ref * s_ref[i];
    }

    //acc cost
    for(int i = 1; i < N_-1; ++i)
    {
        H_triplets.emplace_back(i-1, i-1,  2.0 * w_acc);

        H_triplets.emplace_back(i,   i,    8.0 * w_acc);

        H_triplets.emplace_back(i+1, i+1,  2.0 * w_acc);

        H_triplets.emplace_back(i-1, i,   -4.0 * w_acc);

        H_triplets.emplace_back(i, i-1,   -4.0 * w_acc);

        H_triplets.emplace_back(i, i+1,   -4.0 * w_acc);

        H_triplets.emplace_back(i+1, i,   -4.0 * w_acc);

        H_triplets.emplace_back(i-1, i+1, 2.0 * w_acc);

        H_triplets.emplace_back(i+1, i-1, 2.0 * w_acc);
    }

    //Jerk cost
    Eigen::MatrixXd D_jerk = Eigen::MatrixXd::Zero(N_-3,N_);
    for(int i = 0; i < N_ - 3; ++i) {
        D_jerk(i,i) = -1.0;
        D_jerk(i,i+1) = 3.0;
        D_jerk(i,i+2) = -3.0;
        D_jerk(i,i+3) = 1.0;
    }

    Eigen::MatrixXd P_jerk = 2 * w_jerk * D_jerk.transpose() * D_jerk;

    for(int i = 0; i < N_; ++i) {
        for(int j = 0; j < N_; ++j) {
           constexpr double kEps = 1e-10;
            if(std::fabs(P_jerk(i,j)) > kEps)
            {
                H_triplets.emplace_back(i,j,P_jerk(i,j));
            }
        }
    }

    const double eps_reg = 1e-6;
    for (int i = 0; i < N_; ++i) {
        H_triplets.emplace_back(i, i, eps_reg);
    }

    H.setFromTriplets(H_triplets.begin(), H_triplets.end());

    //Constraints Matrix
    Eigen::SparseMatrix<OSQPFloat> A(n_constraints,n_vars);
    std::vector<Eigen::Triplet<OSQPFloat>> A_triplets;
    
    for(int i = 0; i < N_; ++i) {
        A_triplets.emplace_back(i,i,1.0);
    }

    for(int i = N_; i < 2 * N_ - 1; ++i) {
        int col = i - N_;
        A_triplets.emplace_back(i, col, -1.0);
        A_triplets.emplace_back(i, col + 1, 1.0);
    }

    for(int i = 2 * N_ - 1; i < 3 * N_ - 3; ++i) {
        int col = i - (2 * N_ - 1);
        A_triplets.emplace_back(i, col, 1.0);
        A_triplets.emplace_back(i, col + 1, -2.0);
        A_triplets.emplace_back(i, col + 2, 1.0);
    }

    for(int i = 3 * N_ - 3; i < n_constraints; ++i) {
        int col = i - (3 * N_ - 3);
        A_triplets.emplace_back(i, col, -1.0);
        A_triplets.emplace_back(i, col + 1, 3.0);
        A_triplets.emplace_back(i, col + 2, -3.0);
        A_triplets.emplace_back(i, col + 3, 1.0);
    }   

    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

   //Bounds
    Eigen::VectorXd l(n_constraints);
    Eigen::VectorXd u(n_constraints);

    l(0) = 0.0;
    u(0) = 0.0;

    for(int i = 1; i < N_; ++i) {
        l(i) = s_ref[i] - 0.6;
        u(i) = s_ref[i] + 0.6;
    }

    for(int i = N_; i < 2 * N_ - 1; ++i) {
        l(i) = 0.0;
        u(i) = v_max * dt_;
    }

    for(int i = 2 * N_ - 1; i < 3 * N_ - 3; ++i) {
        l(i) = a_min * dt_ * dt_;
        u(i) = a_max * dt_ * dt_;
    }

    for(int i = 3 * N_ - 3; i < n_constraints; ++i) {
        l(i) = -jerk_max * dt_ * dt_ * dt_;
        u(i) = jerk_max * dt_ * dt_ * dt_;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(true);
    
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(l);
    solver.data()->setUpperBound(u);

    if(!solver.initSolver()) {
        std::cerr << "Failed to initialize OSQP solver!" << std::endl;
        return {};
    }

    //Result
    auto status = solver.solveProblem();

    if(status != OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cerr
            << "OSQP solve failed"
            << std::endl;

        return {};
    }
   
    Eigen::VectorXd s_opt = solver.getSolution();
    std::cout << "Optimal Speed Values:\n" << s_opt.transpose() << std::endl;

    // Return the optimal speed values
    std::vector<double> result;
    for(size_t i = 0; i < s_opt.size(); ++i) {
        result.push_back(s_opt(i));
    }
    
    std::ofstream file("../output/speed_qp.csv");

    file << "t,s_ref,s_qp,v,a,jerk\n";

    for(int i = 0; i < N_; ++i)
    {
        double t = i * dt_;

        double v = 0.0;
        double a = 0.0;

        if(i < N_ - 1)
        {
            v = (s_opt(i+1) - s_opt(i)) / dt_;
        }

        
        if(i > 0 && i < N_ - 1)
        {
            a = (s_opt(i+1) - 2.0*s_opt(i) + s_opt(i-1)) / (dt_*dt_);
        }

        double jerk = 0.0;
        if(i > 1 && i < N_ - 2)
        {
            jerk = (s_opt(i+2) - 3.0 * s_opt(i+1) + 3.0 * s_opt(i) - s_opt(i-1)) / (dt_*dt_*dt_);
        }

        file
            << t << ","
            << s_ref[i] << ","
            << s_opt(i) << ","
            << v << ","
            << a << ","
            << jerk
            << "\n";
    }

    file.close();
    return result;
}