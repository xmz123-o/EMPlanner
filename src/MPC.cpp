#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>

int main() {
    const int N = 30; //Prediction horizon
    const int x_ref = 10;

    const int nx = N + 1; //number of states
    const int nu = N; //number of control inputs
    const int n_vars = nx + nu;
    const int n_constraints = 3*N + 1;

    Eigen::SparseMatrix<OSQPFloat> hessian(n_vars, n_vars);  //P
    Eigen::VectorXd gradient(n_vars);  //Q

    std::vector<Eigen::Triplet<OSQPFloat>> P_triplets;
    for(int i = 0; i < nx; ++i) {
        P_triplets.emplace_back(i,i,2.0); // State cost
    }
    for(int i = nx; i < n_vars; ++i) {
        P_triplets.emplace_back(i,i,5.0); // Control cost
    }

    hessian.setFromTriplets(P_triplets.begin(), P_triplets.end());

    gradient.setZero();
    for(int i = 0; i < nx; ++i) {
        gradient(i) = -2.0 * x_ref;
    }

    Eigen::SparseMatrix<OSQPFloat> linearMatrix(n_constraints, n_vars); //A
    std::vector<Eigen::Triplet<OSQPFloat>> A_triplets;

    A_triplets.emplace_back(0,0,1.0); // Initial state constraint

    for(int i = 0; i < N; ++i) {
        int row = i+1;
        A_triplets.emplace_back(row, i+1, 1.0); // x_{k+1}
        A_triplets.emplace_back(row, i, -1.0); // -x_k
        A_triplets.emplace_back(row, nx + i, -1.0); // -u_k
    }

    for(int i = 0; i < N; ++i) {
        int row = N + 1 + i;
        A_triplets.emplace_back(row,nx + i,1.0);
    }

    int row = 2*N + 1;
    A_triplets.emplace_back(row,nx,1.0);

    for(int i = 1; i < N; ++i) {
        int row = 2*N + 1 + i;
        A_triplets.emplace_back(row,nx + i,1.0);
        A_triplets.emplace_back(row,nx + i - 1,-1.0);
    }

    linearMatrix.setFromTriplets(A_triplets.begin(), A_triplets.end());

    Eigen::VectorXd lowerBound (n_constraints); //l
    Eigen::VectorXd upperBound (n_constraints); //u

    lowerBound(0) = 0.0;
    upperBound(0) = 0.0;

    for(int i = 1; i <= N; ++i) {
        lowerBound(i) = 0.0;
        upperBound(i) = 0.0;
    }

    for(int i = N + 1; i <= 2*N ; ++i) {
        lowerBound(i) = -1.0;
        upperBound(i) = 1.0;
    }

    for(int i = 2*N + 1; i < n_constraints; ++i) {
        lowerBound(i) = -0.2;
        upperBound(i) = 0.2;
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(true);
    
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    if(!solver.initSolver()) {
        std::cout << "Failed to initialize solver" << std::endl;
        return 1;
    }

    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cout << "solve failed" << std::endl;
        return 1;
    }

    auto solution = solver.getSolution();
    std::cout<<"States: "<< solution.head(nx).transpose()<<std::endl;
    std::cout<<"Controls: "<< solution.tail(nu).transpose()<<std::endl;

    return 0;
}