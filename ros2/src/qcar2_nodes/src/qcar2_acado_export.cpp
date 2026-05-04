#include <acado/acado_code_generation.hpp>
#include <acado_toolkit.hpp>

#include <cmath>
#include <iostream>

int main()
{
  USING_NAMESPACE_ACADO

  const double Ts = 0.10;
  const int Npred = 12;
  const double Tf = Ts * static_cast<double>(Npred);

  const double l = 0.25725;
  const double v_min = -0.20;
  const double v_max = 0.45;
  const double omega_s_max = 2.0;
  const double varphi_max = 0.5236;

  DifferentialState x;
  DifferentialState y;
  DifferentialState theta;
  DifferentialState varphi;

  Control v;
  Control omega_s;

  DifferentialEquation f;
  f << dot(x) == v * cos(theta);
  f << dot(y) == v * sin(theta);
  f << dot(theta) == (v / l) * tan(varphi);
  f << dot(varphi) == omega_s;

  Function h;
  h << x;
  h << y;
  h << theta;
  h << varphi;
  h << v;
  h << omega_s;

  Function hN;
  hN << x;
  hN << y;
  hN << theta;
  hN << varphi;

  DMatrix W = eye<double>(6);
  W(0, 0) = 80.0;
  W(1, 1) = 80.0;
  W(2, 2) = 8.0;
  W(3, 3) = 2.0;
  W(4, 4) = 5.0;
  W(5, 5) = 1.0;

  DMatrix WN = eye<double>(4);
  WN(0, 0) = 200.0;
  WN(1, 1) = 200.0;
  WN(2, 2) = 20.0;
  WN(3, 3) = 5.0;

  OCP ocp(0.0, Tf, Npred);
  ocp.minimizeLSQ(W, h);
  ocp.minimizeLSQEndTerm(WN, hN);

  ocp.subjectTo(f);
  ocp.subjectTo(v_min <= v <= v_max);
  ocp.subjectTo(-omega_s_max <= omega_s <= omega_s_max);
  ocp.subjectTo(-varphi_max <= varphi <= varphi_max);

  OCPexport mpc(ocp);
  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_RK4);
  mpc.set(NUM_INTEGRATOR_STEPS, 4 * Npred);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);
  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, YES);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);
  mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

  const char * out_dir = "acado_qcar2_nlmpc";
  const returnValue status = mpc.exportCode(out_dir);
  if (status != SUCCESSFUL_RETURN) {
    std::cerr << "Failed to export ACADO solver. returnValue=" << status << std::endl;
    return 1;
  }

  std::cout << "ACADO solver exported to " << out_dir << std::endl;
  return 0;
}
