#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iosqp/iosqp.hpp>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

// OsqpEigen::Solver solver;
osqp::IOSQP qpSolver_;
TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}


// 多项式阶乘
int TrajectoryGeneratorWaypoint::Factorial(int x){
  int fac = 1;
  for(int i=x; i>0; i--){
    fac = fac*i;
  }
  return fac;
}

// 获取hessian矩阵
void TrajectoryGeneratorWaypoint::GetHessian(const int n_seg,
                                             const int d_order,
                                             const Eigen::VectorXd Time,
                                             Eigen::SparseMatrix<double> &hessian){
  int p_order = 2*d_order-1;
  int p_num1d = p_order+1;
  
  hessian.resize(n_seg*p_num1d, n_seg*p_num1d);
  hessian.setZero();

  for(int k=0; k<n_seg; k++){

    for(int i=d_order; i<p_num1d; i++){
      for(int j=d_order; j<p_num1d; j++){
        double value = 1.0*Factorial(i)/Factorial(i-d_order)*Factorial(j)/Factorial(j-d_order)/(i+j-2*d_order+1)*pow(Time(k), i+j-2*d_order+1);
        hessian.coeffRef(k*p_num1d+i, k*p_num1d+j) = value;
      }
    }
  }
}

// 在线性约束矩阵指定位置插入系数
void TrajectoryGeneratorWaypoint::InsertCoeff(const int row,
                                              const int col,
                                              const int d_order,
                                              const double t,
                                              const int num_constraints,
                                              const bool reverse,
                                              Eigen::SparseMatrix<double> &linearMatrix){
  int p_num1d = 2*d_order;

  Eigen::MatrixXd coeff(d_order, p_num1d);
  switch (d_order){
    case 3:
      coeff << 1.0, 1.0*t, 1.0*pow(t,2), 1.0*pow(t,3), 1.0*pow(t,4),  1.0*pow(t,5),
               0.0, 1.0,   2.0*t,        3.0*pow(t,2), 4.0*pow(t,3),  5.0*pow(t,4),
               0.0, 0.0,   2.0,          6.0*t,        12.0*pow(t,2), 20.0*pow(t,3);
      break;
  
    case 4:
      coeff << 1.0, 1.0*t, 1.0*pow(t,2), 1.0*pow(t,3), 1.0*pow(t,4),  1.0*pow(t,5),  1.0*pow(t,6),   1.0*pow(t,7),
               0.0, 1.0,   2.0*t,        3.0*pow(t,2), 4.0*pow(t,3),  5.0*pow(t,4),  6.0*pow(t,5),   7.0*pow(t,6),
               0.0, 0.0,   2.0,          6.0*t,        12.0*pow(t,2), 20.0*pow(t,3), 30.0*pow(t,4),  42.0*pow(t,5),
               0.0, 0.0,   0.0,          6.0,          24.0*t,        60.0*pow(t,2), 120.0*pow(t,3), 210.0*pow(t,4);
      break;

    default:
      cout << "请使用minimum snap或minimum jerk" << endl;
      break;
  }

  if(reverse){
    coeff = -coeff;
  }

  for(int i=0; i<num_constraints; i++){
    for(int j=0; j<p_num1d; j++){
      linearMatrix.coeffRef(row+i, col+j) = coeff(i,j);
    }
  }
}

// 获取等式约束矩阵
void TrajectoryGeneratorWaypoint::GetlinearConstraintsMatrix(const int n_seg,
                                                             const int d_order,
                                                             const Eigen::VectorXd Time,
                                                             Eigen::SparseMatrix<double> &linearMatrix){
  int p_order = 2*d_order-1;
  int p_num1d = p_order+1;
  linearMatrix.resize(2*d_order + (n_seg-1)*(d_order+1), p_num1d*n_seg);

  // 起点约束
  int row=0;
  int col=0;
  InsertCoeff(row, col, d_order, 0, d_order, false, linearMatrix);

  // 终点约束
  row += d_order;
  col = (n_seg-1)*p_num1d;
  InsertCoeff(row, col, d_order, Time(n_seg-1), d_order, false, linearMatrix);

  // 中间点位置约束
  row += d_order;
  for(int k=0; k<n_seg-1; k++){
    InsertCoeff(row+k, k*p_num1d, d_order, Time(k), 1, false, linearMatrix);
  }

  // 中间点连续性约束
  row += n_seg-1;
  for(int k=0; k<n_seg-1; k++){
    InsertCoeff(row, k*p_num1d, d_order, Time(k), d_order, false, linearMatrix);
    InsertCoeff(row, (k+1)*p_num1d, d_order, 0, d_order, true, linearMatrix);
    row += d_order;
  }
}


Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/

  // solver.data()->setNumberOfVariables( m * p_num1d );
  // solver.data()->setNumberOfConstraints(d_order*2 + (m - 1)*(d_order+1) );

  // // 设置Hessian矩阵
  Eigen::SparseMatrix<double> hessian;
  GetHessian(m, d_order, Time, hessian);
  // if(!solver.data()->setHessianMatrix(hessian)){
  //     cout << "设置hession矩阵失败";
  //     return Eigen::MatrixXd::Zero(1,1);
  // }


  // // 设置线性约束矩阵
  Eigen::SparseMatrix<double> linearMatrix;
  GetlinearConstraintsMatrix(m, d_order, Time, linearMatrix);
  // if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)){
  //     cout << "设置Linear矩阵失败";
  //     return Eigen::MatrixXd::Zero(1,1);
  // }

  // 设置梯度约束
  // Eigen::VectorXd gradient(p_num1d * m);
  // gradient.setZero();
  // if(!solver.data()->setGradient(gradient)){
  //     cout << "梯度设置失败" <<endl;
  // }


  // // 设置边界，求解问题
  Eigen::VectorXd lowbound = VectorXd::Zero(d_order*2 + (m-1)*(d_order+1));
  Eigen::VectorXd upbound = VectorXd::Zero(d_order*2 + (m-1)*(d_order+1));

  Eigen::VectorXd q = VectorXd::Zero(m*p_num1d);

  // solver.data()->setLowerBound(lowbound);
  // solver.data()->setUpperBound(upbound);

  //初始化求解器
  // if(!solver.isInitialized()){
  //     solver.initSolver();
  // }


  for(int dim=0; dim<3; dim++){
    VectorXd waypoints = Path.col(dim);

    // 起点位置
    lowbound(0) = waypoints(0);
    upbound(0) = waypoints(0);
    lowbound(1) = Vel(0, dim);
    upbound(1) = Vel(0, dim);
    lowbound(2) = Acc(0, dim);
    upbound(2) = Acc(0, dim);
    // jerk=0


    // 终点位置
    lowbound(d_order) = waypoints(m);
    upbound(d_order) = waypoints(m);
    lowbound(d_order+1) = Vel(1, dim);
    upbound(d_order+1) = Vel(1, dim);
    lowbound(d_order+2) = Acc(1, dim);
    upbound(d_order+2) = Acc(1, dim);
    // jerk=0

    // 固定中间节点位置
    for(int i=0; i<m-1; i++){
      lowbound(2*d_order+i) = waypoints(i+1);
      upbound(2*d_order+i) = waypoints(i+1);
    }

    // 更新边界
    // solver.updateBounds(lowbound,upbound);

    qpSolver_.setMats(hessian, q, linearMatrix, lowbound, upbound);
    qpSolver_.solve();
    // 求解
    if(!(int)qpSolver_.getStatus()){
      cout << "error code: " << endl;
      ROS_ERROR("fail to solve QP!");
      return Eigen::MatrixXd::Zero(1,1);
    }

    // solver.solve();

    // Eigen::VectorXd poly_coef_1d = solver.getSolution();
    // MatrixXd poly_coef_1d_t = poly_coef_1d.transpose();
    Eigen::VectorXd poly_coef_1d = qpSolver_.getPrimalSol();
    MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(poly_coef_1d.data(), 1, m*p_num1d);

    for(int k=0; k<m; k++){
      // 建议 block 对 block，否则可能会有问题
      // PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = poly_coef_1d_t.block(0, k*p_num1d, 1, p_num1d);
      PolyCoeff.block(k, dim*p_num1d, 1, p_num1d) = solMat.block(0, k*p_num1d, 1, p_num1d);
    }
  }

  // 每次调用之后需要清理变量
  // solver.data()->clearHessianMatrix();
  // solver.data()->clearLinearConstraintsMatrix();
  // solver.clearSolverVariables();
  // solver.clearSolver();

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}