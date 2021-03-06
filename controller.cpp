#include "controller.h"
//#include <iomanip>

#define dof 2
#define mode 2 //1:joint control //2:task control //3:new con

void ArmController::compute()
{
  //get joint value in 2D
  for (int i = 0; i < dof; ++i)
  {
    q2D_(i) = q_(i);
    q2Ddot_(i) = qdot_(i);
    q2Dext_(i) = qext_(i);
    current2D_(i) = current_(i);
  }
  //control initialize
  if (initialize_control == false)
  {
    q2D_pre = q2D_;
    q2Dext_pre = q2Dext_;
    q2Ddot_pre.setZero();
    q2Ddotext_pre.setZero();

    initialize_control = true;
  }

  //cout << q2D_<< endl << endl;
  //calculate vel,acc
  q2Dddot_ = (q2Ddot_ - q2Ddot_pre) / 0.001;
  q2Ddotext_ = (q2Dext_ - q2Dext_pre) / 0.001;
  q2Dddotext_ = (q2Ddotext_ - q2Ddotext_pre) / 0.001;
  //filtering
  for (int i = 0; i < dof; ++i)
  {
    q2Ddot_f(i) = lowpassfilter(0.001, 600.0, q2Ddot_(i), q2Ddot_f_pre(i));
    q2Dddot_f(i) = lowpassfilter(0.001, 200.0, q2Dddot_(i), q2Dddot_f_pre(i));
    q2Ddotext_f(i) = lowpassfilter(0.001, 600.0, q2Ddotext_(i), q2Ddotext_f_pre(i));
    q2Dddotext_f(i) = lowpassfilter(0.001, 600.0, q2Dddotext_(i), q2Dddotext_f_pre(i));
  }
  q2Ddot_f_pre = q2Ddot_f;
  q2Dddot_f_pre = q2Dddot_f;
  q2Ddotext_f_pre = q2Ddotext_f;
  q2Dddotext_f_pre = q2Dddotext_f;

  //get dynamics and jac
  A = getA(q2D_);
  M = A+B;
  J = getJ(q2D_);
  J_T = J.transpose();
  C = getC(q2D_, q2Ddot_f);
  C_T = C.transpose();
  Lambda = getLambda(J, M);
  Lambda_m = getLambda(J, B);
  Lambda_l = getLambda(J, A);
  //cout << A << endl << endl;
  x_(0) = 0.3 * cos(q2D_(0)) + 0.3 * cos(q2D_(0) + q2D_(1));
  x_(1) = -0.3 * sin(q2D_(0)) - 0.3 * sin(q2D_(0) + q2D_(1));
  x_dot_ = J * q2Ddot_;
  //q_ik = ik(x_);
  //cout << Lambda << "\t" << Lambda_m << endl << endl;

  //control after 4 seconds
  if (mode == 1)
  {
    if (play_time_ == 6.0)
    {
      control_start_time_ = play_time_;
      torque_desired_.setZero();
      q2D_init_ = q2D_;
      q2D_target(0) = q2D_init_(0) + 30.0 * DEG2RAD;
      q2D_target(1) = q2D_init_(1) + 30.0 * DEG2RAD;
      control_time_ = 1.0;
    }
    else if (play_time_ < 4.0)
    {
      control_start_time_ = play_time_;
      torque_desired_.setZero();
      q2D_init_ = q2D_;
      q2D_target(0) = 10.0 * DEG2RAD;
      q2D_target(1) = 10.0 * DEG2RAD;
    }
    else
    {
    }

    //control trajectory
    if (play_time_ < 6.0 && play_time_ >= 4.0)
    {
      q2D_desired_(0) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2D_desired_(1) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Ddot_desired_(0) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Ddot_desired_(1) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Dddot_desired_(0) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Dddot_desired_(1) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
    }
    else if (play_time_ >= 6.0)
    {
      q2D_desired_(0) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2D_desired_(1) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Ddot_desired_(0) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Ddot_desired_(1) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Dddot_desired_(0) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Dddot_desired_(1) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + control_time_, q2D_init_(1), q2D_target(1), 0.0, 0.0);
    }
    else
    {
      q2D_desired_ = q2D_init_;
    }
  }
  else if (mode == 2 || mode == 3)
  {
    if (play_time_ == 6.0)
    {
      control_start_time_ = play_time_;
      torque_desired_.setZero();
      x_init_ = x_;
      x_target(0) = x_init_(0) - 0.15;
      x_target(1) = x_init_(1) - 0.0;
      control_time_ = 1.0;
    }
    else if (play_time_ < 4.0)
    {
      control_start_time_ = play_time_;
      torque_desired_.setZero();
      q2D_init_ = q2D_;
      q2D_target(0) = 10.0 * DEG2RAD;
      q2D_target(1) = 10.0 * DEG2RAD;
    }
    else
    {
    }

    //control trajectory
    if (play_time_ < 6.0 && play_time_ >= 4.0)
    {
      q2D_desired_(0) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2D_desired_(1) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Ddot_desired_(0) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Ddot_desired_(1) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
      q2Dddot_desired_(0) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(0), q2D_target(0), 0.0, 0.0);
      q2Dddot_desired_(1) = DyrosMath::cubicDDot(play_time_, control_start_time_, control_start_time_ + 1.0, q2D_init_(1), q2D_target(1), 0.0, 0.0);
    }
    else if (play_time_ >= 6.0)
    {
      x_desired_(0) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + control_time_, x_init_(0), x_target(0), 0.0, 0.0);
      x_desired_(1) = DyrosMath::cubic(play_time_, control_start_time_, control_start_time_ + control_time_, x_init_(1), x_target(1), 0.0, 0.0);
      x_dot_desired_(0) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + control_time_, x_init_(0), x_target(0), 0.0, 0.0);
      x_dot_desired_(1) = DyrosMath::cubicDot(play_time_, control_start_time_, control_start_time_ + control_time_, x_init_(1), x_target(1), 0.0, 0.0);
    }
    else
    {
      x_desired_ = x_init_;
    }
  }

  kp_mod(0) = alpha / (alpha + abs(torque2D_dist_est_mod(0))) * Kp;
  kp_mod(1) = alpha / (alpha + abs(torque2D_dist_est_mod(1))) * Kp;
  kv_mod(0) = alpha / (alpha + abs(torque2D_dist_est_mod(0))) * Kv;
  kv_mod(1) = alpha / (alpha + abs(torque2D_dist_est_mod(1))) * Kv;

  if (play_time_ >= 6.0)
  {
    if(mode == 1){
      //controller
      torque2D_desired_ = (Kp * (q2D_desired_ - q2D_) + Kv * (q2Ddot_desired_ - q2Ddot_f));
      torque_desired_(0) = alpha / (alpha + abs(torque2D_dist_est_mod(0))) * torque2D_desired_(0);
      torque_desired_(1) = alpha / (alpha + abs(torque2D_dist_est_mod(1))) * torque2D_desired_(1);
      //torque_desired_(0) = pow(alpha,abs(torque2D_dist_est_mod(0)))*torque2D_desired_(0);
      //torque_desired_(1) = pow(alpha,abs(torque2D_dist_est_mod(1)))*torque2D_desired_(1);
      //torque_desired_(0) = torque2D_desired_(0);
      //torque_desired_(1) = torque2D_desired_(1);
    }
    else if(mode == 2){
      fstar = (Kpx * (x_desired_ - x_) + Kvx * (x_dot_desired_ - x_dot_));
      torque2D_desired_ = J_T * Lambda * fstar;
      torque_desired_(0) = torque2D_desired_(0);
      torque_desired_(1) = torque2D_desired_(1);

      fstar_pre = fstar;
    }
    else if(mode == 3){
      fstar_l = (Kpxl * (x_desired_ - x_) + Kvxl * (x_dot_desired_ - x_dot_));
      fstar_h = (Kpxh * (x_desired_ - x_) + Kvxh * (x_dot_desired_ - x_dot_));
      for(int i=0; i<dof; ++i){
        fstar_lf(i) = lowpassfilter(0.001, sqrt(20000.0/A(0,0))/20.0, fstar_l(i), fstar_lf_pre(i));//lowpassfilter(0.001, sqrt(20000.0/A(0,0)), fstar_l(i), fstar_lf_pre(i));
        fstar_hf(i) = highpassfilter(0.001, sqrt(20000.0/A(0,0))/20.0, fstar_h(i), fstar_h_pre(i), fstar_hf_pre(i));
      }
      torque2D_desired_ = J_T * Lambda_l * fstar_lf + J_T * Lambda_m * fstar_hf;
      torque_desired_(0) = torque2D_desired_(0);
      torque_desired_(1) = torque2D_desired_(1);

      fstar_h_pre = fstar_h;
      fstar_l_pre = fstar_l;
      fstar_hf_pre = fstar_hf;
      fstar_lf_pre = fstar_lf;
    }

    torque_desired_(2) = 0.0;
  }
  else if (play_time_ >= 4.0 && play_time_ < 6.0)
  {
    //controller
    torque2D_desired_ = 900.0 * (q2D_desired_ - q2D_) + 50.0 * (q2Ddot_desired_ - q2Ddot_f);
    torque_desired_(0) = torque2D_desired_(0);
    torque_desired_(1) = torque2D_desired_(1);
    torque_desired_(2) = 0.0;
  }
  else
  {
    torque_desired_.setZero();
  }
  Gamma_L_est_ = B * q2Dddot_f;
  //friction estimate //low velocity should be reconsidered
  for (int i = 0; i < dof; ++i){//should be modified depending on the exp condition. e.g. temperature, weight, etc
    if (abs(current2D_(i)) <= 2.2 && abs(q2Ddot_f(i)) < 0.005)
      torque2D_fric_est(i) = current2D_(i);
    // else if (abs(current2D_(i)) > 9.0 && q2Ddot_f(i) > -0.5 && q2Ddot_f(i) < 0.0)
    //   torque2D_fric_est(i) = -2;
    // else if (abs(current2D_(i)) > 9.0 && q2Ddot_f(i) < 0.5 && q2Ddot_f(i) > 0.0)
    //   torque2D_fric_est(i) = 2;
    else{
      // if(current2D_(i) > 0.0 && q2Ddot_f(i) < 0.0)
      //   torque2D_fric_est(i) = -2;
      // else if(current2D_(i) < 0.0 && q2Ddot_f(i) > 0.0)
      // torque2D_fric_est(i) = 2;
      //else
        torque2D_fric_est(i) = 11 * q2Ddot_f(i);
    }
  }
  //momentum observer
  momentum_ = (A + B) * q2Ddot_f;
  momentum_dot_est_ = current2D_ + C_T * q2Ddot_f + torque2D_dist_est - torque2D_Gravity - torque2D_fric_est;
  momentum_est_ = momentum_est_ + momentum_dot_est_ * 0.001;
  momentum_err = momentum_ - momentum_est_;
  for (int i = 0; i < dof; ++i)
  { //modifying mob
    if (abs(momentum_err(i)) < 0.5 && abs(q2Ddot_desired_(i)) > 0.0)
      momentum_err(i) = 0.0;

    if (abs(momentum_err(i)) < 0.3)
      momentum_err(i) = 0.0;
  }
  torque2D_dist_est = Lgain_ * (momentum_ - momentum_est_);
  torque2D_dist_est_mod = Lgain_ * momentum_err;

  // mom_s = (A(0,0))*q2Ddot_(0);
  // mom_s_dot = torque2D_(0) + tor_dist;
  // mom_s_est = mom_s_est + mom_s_dot*0.001;
  // tor_dist = Lgain_*(mom_s - mom_s_est);

  //momentum observer
  // momentum_ = B * q2Ddot_;
  // momentum_dot_est_ =  current2D_ -Gamma_L_est_ -C*q2Ddot_ + torque2D_dist_est - torque2D_Gravity - torque2D_fric_est;
  // momentum_est_ = momentum_est_ + momentum_dot_est_ * 0.001;
  // momentum_err = momentum_ - momentum_est_;
  // for (int i = 0; i < dof; ++i){
  //   if (abs(momentum_err(i)) < 0.2 && abs(q2Ddot_desired_(i)) > 0.0)
  //     momentum_err(i) = 0.0;

  //   if(abs(momentum_err(i)) < 0.02 && abs(q2Ddot_(i)) > 0.001){}
  // }
  // torque2D_dist_est = Lgain_ * (momentum_ - momentum_est_);
  // torque2D_dist_est_mod  = Lgain_ * momentum_err;
  //filtering
  for (int i = 0; i < dof; ++i)
  {
    torque2D_dist_est_f(i) = lowpassfilter(0.001, 100.0, torque2D_dist_est(i), torque2D_dist_est_f_pre(i));
  }
  torque2D_dist_est_f_pre = torque2D_dist_est_f;
  force_ext_est_ = J_T.inverse() * torque2D_dist_est;

  //pre value save
  q2D_pre = q2D_;
  q2Ddot_pre = q2Ddot_;
  q2Dddot_pre = q2Dddot_;
  q2Dext_pre = q2Dext_;
  q2Ddotext_pre = q2Ddotext_;
  q2Dddotext_pre = q2Dddotext_;
}

MatrixXd ArmController::getA(Vector2d q){
  MatrixXd Amat;
  Amat.resize(2, 2);
  double m1 = 8.19054356366;
  //double m2 = 7.63361475695; //no bar
  //double m2 = 9.92016314806; //only bar
  //double m2 = 19.92016314806; //add 10kg
  double m2 = 29.92016314806; //add 20kg
  double l1 = 0.3;
  double l2 = 0.3;
  double lc1 = sqrt(0.120790326 * 0.120790326 + 0.0000340612 * 0.0000340612);
  //double lc2 = sqrt(0.1193127722*0.1193127722 + 0.0000133339*0.0000133339); //no bar
  //double lc2 = sqrt(0.1607810504 * 0.1607810504 + 0.0000910835 * 0.0000910835); // only bar
  //double lc2 = sqrt(0.2306695089*0.2306684095847 + 0.0000453592*0.0000453592); //add 10kg
  double lc2 = sqrt(0.2538413381*0.2538413381 + 0.0000301991*0.0000301991); //add 20kg
  double Izz1 = 0.17425889349;
  //double Izz2 = 0.15006242579; //no bar
  //double Izz2 = 0.20813323401; //only bar
  //double Izz2 = 0.39309231441; //add 10kg
  double Izz2 = 0.51353221268; //add 20kg

  Amat(0, 0) = m1 * lc1 * lc1 + Izz1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(q(1)));
  Amat(0, 1) = Izz2 + m2 * (lc2 * lc2 + l1 * lc2 * cos(q(1)));
  Amat(1, 0) = Amat(0, 1);
  Amat(1, 1) = lc2 * lc2 * m2 + Izz2;

  return Amat;
}

MatrixXd ArmController::getC(Vector2d q, Vector2d qdot){
  double h = 2e-12;

  Vector2d q_new = q;
  MatrixXd C(dof, dof);
  MatrixXd C1(dof, dof);
  C1.setZero();
  MatrixXd C2(dof, dof);
  C2.setZero();
  MatrixXd H_origin(dof, dof), H_new(dof, dof);
  MatrixXd m[dof];
  double b[dof][dof][dof];
  H_origin = getA(q); //CompositeRigidBodyAlgorithm(*model, q, H_origin, true);

  for (int i = 0; i < dof; i++)
  {
    q_new = q;
    q_new(i) += h;
    H_new = getA(q_new); //CompositeRigidBodyAlgorithm(*model, q_new, H_new, true);
    m[i].resize(dof, dof);
    m[i] = (H_new - H_origin) / h;
  }

  for (int i = 0; i < dof; i++)
    for (int j = 0; j < dof; j++)
      for (int k = 0; k < dof; k++)
        b[i][j][k] = 0.5 * (m[k](i, j) + m[j](i, k) - m[i](j, k));

  C.setZero();
  for (int i = 0; i < dof; i++)
    for (int j = 0; j < dof; j++)
      C1(i, j) = b[i][j][j] * qdot(j);

  for (int k = 0; k < dof; k++)
    for (int j = 0; j < dof; j++)
      for (int i = 1 + j; i < dof; i++)
        C2(k, j) += 2.0 * b[k][j][i] * qdot(i);

  C = C1 + C2;

  return C;
}

MatrixXd ArmController::getC2(Vector2d q, Vector2d qdot){
  MatrixXd C(dof, dof);
  MatrixXd C1(dof, dof);
  C1.setZero();
  MatrixXd C2(dof, dof);
  C2.setZero();

  double m1 = 8.19054356366;
  //double m2 = 7.63361475695; //no bar
  double m2 = 9.92016314806; //only bar
  //double m2 = 19.92016314806; //add 10kg
  //double m2 = 29.92016314806; //add 20kg
  double l1 = 0.3;
  double l2 = 0.3;
  double lc1 = sqrt(0.120790326 * 0.120790326 + 0.0000340612 * 0.0000340612);
  //double lc2 = sqrt(0.1193127722*0.1193127722 + 0.0000133339*0.0000133339); //no bar
  double lc2 = sqrt(0.1607810504 * 0.1607810504 + 0.0000910835 * 0.0000910835); // only bar
  //double lc2 = sqrt(0.2306695089*0.2306684095847 + 0.0000453592*0.0000453592); //add 10kg
  //double lc2 = sqrt(0.2538413381*0.2538413381 + 0.0000301991*0.0000301991); //add 20kg
  double Izz1 = 0.17425889349;
  //double Izz2 = 0.15006242579; //no bar
  double Izz2 = 0.20813323401; //only bar
  //double Izz2 = 0.39309231441; //add 10kg
  //double Izz2 = 0.51353221268; //add 20kg

  C1(0, 1) = -l1 * lc2 * m2 * sin(q(1)) * qdot(1);
  C1(1, 0) = l1 * lc2 * m2 * sin(q(1)) * qdot(0);

  C2(0, 0) = -2 * l1 * lc2 * m2 * sin(q(1)) * qdot(1);

  C = C1 + C2;

  return C;
}

MatrixXd ArmController::getJ(Vector2d q){
  MatrixXd Jac;
  Jac.resize(2, 2);
  Jac(0, 0) = -0.3 * sin(q(0)) - 0.3 * sin(q(0) + q(1));
  Jac(0, 1) = -0.3 * sin(q(0) + q(1));
  Jac(1, 0) = -0.3 * cos(q(0)) - 0.3 * cos(q(0) + q(1));
  Jac(1, 1) = -0.3 * cos(q(0) + q(1));

  return Jac;
}

MatrixXd ArmController::getLambda(MatrixXd J, MatrixXd A){
  MatrixXd Linv;
  MatrixXd L;
  Linv.resize(2, 2);
  L.resize(2, 2);

  Linv = J * A.inverse() * J.transpose();
  L = Linv.inverse();

  return L;
}

Vector2d ArmController::ik(Vector2d p){
  double x = p(0);
  double y = p(1);
  double l = 0.3;
  Vector2d q;

  if(q2D_(1) >= 0.0){
    q(1) = acos((x*x + y*y - 2*l*l)/(2*l*l));
    if(q2D_(1) < 0.0 && q(1) > 0.0)
      q(1) = -2*PI + q(1);
    else if(q2D_(1) > 0.0 && q(1) < 0.0)
      q(1) = 2*PI + q(1); 

    q(0) = -atan2(y,x) - atan2(l*sin(q(1)),(l + l*cos(q(1))));
    if(q2D_(0) < 0.0 && q(0) > 0.0)
      q(0) = -2*PI + q(0);
    else if(q2D_(0) > 0.0 && q(0) < 0.0)
      q(0) = 2*PI + q(0);      
  }
  else{
    q(1) = -acos((x*x + y*y - 2*l*l)/(2*l*l));
    if(q2D_(1) < 0.0 && q(1) > 0.0)
      q(1) = -2*PI + q(1);
    else if(q2D_(1) > 0.0 && q(1) < 0.0)
      q(1) = 2*PI + q(1); 

    q(0) = -atan2(y,x) - atan2(l*sin(q(1)),(l + l*cos(q(1))));
    if(q2D_(0) < 0.0 && q(0) > 0.0)
      q(0) = -2*PI + q(0);
    else if(q2D_(0) > 0.0 && q(0) < 0.0)
      q(0) = 2*PI + q(0);  
  }

  return q;
}

double ArmController::lowpassfilter(double dT, double Wc, double X, double preY){
  double tau = 1.0 / Wc; //tau = RC
  double y = tau / (tau + dT) * preY + dT / (tau + dT) * X;
  return y;
}

double ArmController::highpassfilter(double dT, double Wc, double X, double preX, double preY){
  double tau = 1.0/Wc; //tau = RC
  double y = tau/(tau + dT)*(preY + X - preX);
  return y;
}

ArmController::ArmController(){
  A.resize(dof, dof);
  J.resize(2, dof);
  J_T.resize(dof, 2);
  C.resize(dof, dof);
  C2.resize(dof, dof);
  Lambda.resize(2, 2);
  Lambda_m.resize(2, 2);
  Lambda_l.resize(2, 2);

  torque2D_.setZero();
  current2D_.setZero();
  torque2D_Gravity.setZero();
  torque2D_fric_est.setZero();
  torque2D_dist_est.setZero();
  momentum_est_.setZero();
  force_ext_est_.setZero();
  Lgain_ = 30.0;

  q2Dddot_.setZero();

  q2Ddotext_.setZero();
  q2Dddotext_.setZero();

  q2D_pre.setZero();
  q2Ddot_pre.setZero();
  q2Dddot_pre.setZero();
  q2Dext_pre.setZero();
  q2Ddotext_pre.setZero();
  q2Dddotext_pre.setZero();

  q2Ddot_f.setZero();
  q2Ddot_f_pre.setZero();
  q2Dddot_f.setZero();
  q2Dddot_f_pre.setZero();

  q2Ddotext_f.setZero();
  q2Ddotext_f_pre.setZero();
  q2Dddotext_f.setZero();
  q2Dddotext_f_pre.setZero();

  q2D_desired_.setZero();
  q2Ddot_desired_.setZero();
  q2Dddot_desired_.setZero();
  q2D_init_.setZero();
  q2D_target.setZero();
  torque2D_desired_.setZero();

  errorcheck2D.setZero();
  fstar.setZero();
  fstar_l.setZero();
  fstar_h.setZero();
  fstar_pre.setZero();
  fstar_h_pre.setZero();
  fstar_l_pre.setZero();
  fstar_lf.setZero();
  fstar_hf.setZero();
  fstar_lf_pre.setZero();
  fstar_hf_pre.setZero();

  B.resize(dof, dof);
  B.setZero();
  B(0, 0) = 1.4497694;
  B(1, 1) = 0.93038;
  M.resize(dof,dof);
  M.setZero();

  Kp = 1600.0; //high 3600 low 100
  Kv = 80.0;   //high 120 low 20
  Kpx = 100.0;
  Kvx = 20.0;
  Kpxh = 1600.0;
  Kvxh = 80.0;
  Kpxl = 81.0;
  Kvxl = 18.0;

  mom_s = 0.0;
  mom_s_dot = 0.0;
  mom_s_est = 0.0;
  tor_dist = 0.0;

  kp_mod.setZero();
  kv_mod.setZero();

  alpha = 20;
};

ArmController::~ArmController(){};
