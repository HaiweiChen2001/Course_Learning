#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include <osqp/osqp.h>
// #include <OsqpEigen/OsqpEigen.h>

class TrajectoryGeneratorWaypoint {
private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;

        uint8_t * data;
        int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
        int GLXYZ_SIZE, GLYZ_SIZE;
        double resolution, inv_resolution;
        double gl_xl, gl_yl, gl_zl;
        double gl_xu, gl_yu, gl_zu;

        int Factorial(int x);
        void GetHessian(const int n_seg, const int d_order, const Eigen::VectorXd Time, Eigen::SparseMatrix<double> &hessian);

        void InsertCoeff(const int row, const int col, const int d_order, const double t, 
                                                      const int num_constraints, const bool reverse, Eigen::SparseMatrix<double> &linearMatrix);
        void GetlinearConstraintsMatrix(const int n_seg, const int d_order,
                                                                     const Eigen::VectorXd Time, Eigen::SparseMatrix<double> &linearMatrix);
public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        double getObjective();
        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff, int k, double t );
        Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff, int k, double t );
};

#endif
