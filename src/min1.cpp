#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>


int main() {
    Eigen::SparseMatrix<OSQPFloat> hessian(2, 2);  //P
    Eigen::VectorXd gradient(2);  //Q
    Eigen::SparseMatrix<OSQPFloat> linearMatrix(2, 2); //A
    Eigen::VectorXd lowerBound(2); //l
    Eigen::VectorXd upperBound(2); //u

    hessian.insert(0,0) = 2.0;
    hessian.insert(1,1) = 2.0;

    gradient << -2.0,-2.0;
    linearMatrix.insert(0,0) = 1.0;
    linearMatrix.insert(1,1) = 1.0;

    lowerBound << 0.0,0.0;
    upperBound << 1.5,1.5;

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(2);

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

    Eigen::VectorXd QPsolution = solver.getSolution();
    std::cout<<"QPSolution:"<<QPsolution.transpose()<<std::endl;

    return 0;
}