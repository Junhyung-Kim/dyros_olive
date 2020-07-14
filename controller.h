#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "math_type_define.h"
#include <cmath>
#include <math.h>

#define PI 3.1415926536897932384626 

using namespace std;
using namespace Eigen;

class ArmController
{

private:
  MatrixXd getA(Vector2d q);
  MatrixXd getC(Vector2d q, Vector2d qdot);
  MatrixXd getC2(Vector2d q, Vector2d qdot);
  MatrixXd getJ(Vector2d q);
  MatrixXd getLambda(MatrixXd J, MatrixXd A);
  Vector2d ik(Vector2d p);

  MatrixXd A;
  MatrixXd J;
  MatrixXd J_T;
  MatrixXd C;
  MatrixXd C2;
  MatrixXd C_T;
  MatrixXd B;
  MatrixXd M;
  MatrixXd Lambda, Lambda_m, Lambda_l;

  double lowpassfilter(double dT, double Wc, double X, double preY); //sampling time, cutoff freq, input, previous output)
  double highpassfilter(double dT, double Wc, double X, double preX, double preY); //sampling time, cutoff freq, input, previous input, previous output)

  double Kp, Kv;
  double Kpx, Kvx, Kpxh, Kvxh, Kpxl, Kvxl;
  double alpha;

public:

  size_t dof_;

  // Current state
  Vector3d q_;
  Vector3d qdot_;
  Vector3d qext_;
  Vector3d torque_;
  Vector3d current_;
  Vector3d q_desired_; // Control value
  Vector3d q_init_desired_;
  Vector3d torque_desired_;

  //2 dof state internal
  Vector2d q2D_;
  Vector2d q2Ddot_;
  Vector2d q2D_pre;
  Vector2d q2Ddot_pre;
  Vector2d q2Dddot_;
  Vector2d q2Dddot_pre;

  Vector2d q2Ddot_f;
  Vector2d q2Ddot_f_pre;
  Vector2d q2Dddot_f;
  Vector2d q2Dddot_f_pre;
  //2 dof state external
  Vector2d q2Dext_;
  Vector2d q2Ddotext_;
  Vector2d q2Dext_pre;
  Vector2d q2Ddotext_pre;
  Vector2d q2Dddotext_;
  Vector2d q2Dddotext_pre;

  Vector2d q2Ddotext_f;
  Vector2d q2Ddotext_f_pre;
  Vector2d q2Dddotext_f;
  Vector2d q2Dddotext_f_pre;
  //2 dof state torque
  Vector2d torque2D_;
  Vector2d current2D_;
  Vector2d torque2D_Gravity;
  Vector2d torque2D_fric_est;
  Vector2d torque2D_dist_est;
  Vector2d torque2D_dist_est_f;
  Vector2d torque2D_dist_est_f_pre;
  Vector2d torque2D_dist_est_mod;
  Vector2d momentum_est_;
  Vector2d force_ext_est_;
  Vector2d Gamma_L_est_;
  Vector2d momentum_err;
  Vector2d momentum_;
  Vector2d momentum_dot_est_;
  double Lgain_;
  double mom_s;
  double mom_s_dot;
  double mom_s_est;
  double tor_dist;

  //control value
  Vector2d q2D_desired_; 
  Vector2d q2Ddot_desired_;
  Vector2d q2Dddot_desired_; 
  Vector2d q2D_init_;
  Vector2d q2D_target;
  Vector2d torque2D_desired_;
  Vector2d q_ik;

  Vector2d errorcheck2D;
  Vector2d kp_mod, kv_mod;

  Vector2d x_;
  Vector2d x_dot_;
  Vector2d x_desired_;
  Vector2d x_dot_desired_;
  Vector2d x_init_;
  Vector2d x_target;

  Vector2d fstar;
  Vector2d fstar_l;
  Vector2d fstar_h;
  Vector2d fstar_pre;
  Vector2d fstar_l_pre;
  Vector2d fstar_h_pre;
  Vector2d fstar_lf;
  Vector2d fstar_hf;
  Vector2d fstar_lf_pre;
  Vector2d fstar_hf_pre;

  double play_time_;
  double hz_;
  double control_start_time_;
  double control_time_;

public:

  void compute();
  bool initialize_joint = false;
  bool initialize_control = false;

  ArmController();

  ~ArmController();
};

#endif
