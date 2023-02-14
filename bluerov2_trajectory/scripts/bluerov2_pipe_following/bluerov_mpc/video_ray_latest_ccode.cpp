/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn,
 *    Robotics and Perception Group, University of Zurich
 *
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input.

int main()
{
  // Use Acado
  USING_NAMESPACE_ACADO


  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;      // Initial time [s]
  const double t_end = 1.0;        // Time horizon [s]
  const double dt = 0.05;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
   

  // video ray params
    const double f_s = 1.0;    //scaling factor;
    //const double m = 6.1; 
    const double m = 0.5;   // BlueROV2 mass (kg)  
    const double g = 9.81;  // gravitional field strength (m/s^2)
    const double BW_ratio = 0.8; //Bouyancy to weight ratio ()
    const double F_bouy = m * g * BW_ratio; // Bouyancy force (N)
    const double length = 0.36;    // ROV length (m)  
    const double width = 0.35;    // ROV width (m) 
    const double height = 0.23;    // ROV heigth (m)
    const double T_off = 0.1; // Forward thrusters offset (m)
    //const double V_t =       //ROV



      // VideoRay Hyroddynamic Parameters 
    const double X_ud = -1.9404 * f_s; // Added mass in x direction (kg)
    const double Y_vd = -6.0572 * f_s; // Added mass in y direction (kg)
    const double Z_wd = -3.9482 * f_s; // Added mass in z direction (kg)
    const double N_rd = -0.0118 * f_s ; // Added mass for rotation about z direction (kg)

    const double I_xx = 0.02275 * f_s; // Moment of inertia (kg.m^2)
    const double I_yy = 0.02391 * f_s; // Moment of inertia (kg.m^2)
    const double I_zz = 0.02532 * f_s; // Moment of inertia (kg.m^2)

    const double X_u = -0.09460 * f_s; // Linear damping coefficient in x direction (N.s/m)
    const double Y_v  = -5.8745 * f_s; // Linear damping coefficient  in y direction (N.s/m)
    const double Z_w = -3.7020 * f_s; // Linear damping coefficient  in z direction (N.s/m)
    const double N_r = -0.023 * f_s;  // Linear damping coefficient for rotation about z direction (N.s/rad)

    const double X_uc = -6.0418 * f_s; // quadratic damping coefficient in x direction (N.s^2/m^2)
    const double Y_vc = -30.731 * f_s; // quadratic damping coefficient  in y direction (N.s^2/m^2)
    const double Z_wc = -26.357 * f_s; // quadratic damping coefficient  in z direction (N.s^2/m^2)
    const double N_rc = -0.4504 * f_s ; // quadratic damping coefficient for rotation about z direction (N.s^2/rad^2)
    const double eps = 0.001;  // offset to prevent numerical issues of square root




  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  // System variables
  DifferentialState x, y, z;
  DifferentialState u, v, w;
  DifferentialState psi, r;
  Control T_l, T_r, T_u; // Body rate control
  DifferentialEquation f;
  Function h, hN;

  // Parameters with exemplary values. These are set/overwritten at runtime.


  // System Dynamics of drone.
  //f << dot(x) == cos(psi) * u + sin(psi) * v;
  //f << dot(y) == -sin(psi) * u + cos(psi) * v;
  //f << dot(z) == w;
  //f << dot(u) == 100 * (T_l + T_r);
  //f << dot(v) == 0.0;
  //f << dot(w) == 10 * T_u ;
  //f << dot(psi) == r;
  //f << dot(r) == 1000 * (T_l * 0.1 - T_r * 0.1);


  f << dot(x) == cos(psi) * u - sin(psi) * v;
  f << dot(y) == sin(psi) * u +  cos(psi) * v;
  f << dot(z) ==  w;
  f << dot(u) == (T_l + T_r + (m * v + Y_vd * v) * r + (X_u + X_uc *sqrt( u * u + eps) ) * u)/(m - X_ud) ;
  f << dot(v) == (- (m * u + X_ud * u) * r + (Y_v + Y_vc *sqrt( v * v + eps) ) * v)/(m - Y_vd)  ;
  f << dot(w) == (T_u + (Z_w + Z_wc * sqrt(w * w + eps)) * w + (m * g - F_bouy))/(m - Z_wd) ;
  f << dot(psi) ==  r;
  f << dot(r) == (T_l * T_off - T_r * T_off - (m * v - Y_vd * v) * u - (X_ud * u - m * u) * v + (N_r + N_rc * sqrt(r * r + eps)) * r)/(I_zz - N_rd);



  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << x << y << z << u << v << w << psi << r << T_l << T_r << T_u ;
  hN << x << y << z << u << v << w << psi << r;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0, 0) = 10; // x
  Q(1, 1) = 10; // y
  Q(2, 2) = 10; // z
  Q(3, 3) = 1; // u
  Q(4, 4) = 1; // v
  Q(5, 5) = 1; // w
  Q(6, 6) = 2; // psi
  Q(7, 7) = 1;  // r
  Q(8, 8) = 1;  // T_l
  Q(9, 9) = 1;  // T_r
  Q(10, 10) = 1; // T_u


  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0, 0) = Q(0, 0); // x
  QN(1, 1) = Q(1, 1); // y
  QN(2, 2) = Q(2, 2); // z
  QN(3, 3) = Q(3, 3); // u
  QN(4, 4) = Q(4, 4); // v
  QN(5, 5) = Q(5, 5); // w
  QN(6, 6) = Q(6, 6); // psi
  QN(7, 7) = Q(7, 7); // r

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector ref(h.getDim()); // Running cost reference
  ref.setZero();
  ref(0) = 2.0;
  

  DVector rN(hN.getDim()); // End cost reference
  rN.setZero();
  rN(0) = ref(0);
  

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp(t_start, t_end, N);
  if (!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ(Q, h, ref);
    ocp.minimizeLSQEndTerm(QN, hN, rN);
  }
  else
  {
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);
  }

  // Add system dynamics
  ocp.subjectTo(f);
  // Add constraints
  ocp.subjectTo(-m * g * 5.0 <= T_l <= m * g * 5.0);
  ocp.subjectTo(-m * g * 5.0 <= T_r <= m * g * 5.0);
  ocp.subjectTo(-m * g * 5.0  <= T_u <= m * g * 5.0);

  ocp.setNOD(10);

  if (!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo(AT_START, x == 0.0);
    ocp.subjectTo(AT_START, y == 0.0);
    ocp.subjectTo(AT_START, z == 0.0);
    ocp.subjectTo(AT_START, u == 0.0);
    ocp.subjectTo(AT_START, v == 0.0);
    ocp.subjectTo(AT_START, w == 0.0);
    ocp.subjectTo(AT_START, psi == 0.0);
    ocp.subjectTo(AT_START, r == 0.0);
    ocp.subjectTo(AT_START, T_l == 0.0);
    ocp.subjectTo(AT_START, T_r == 0.0);
    ocp.subjectTo(AT_START, T_u == 0.0);

  
    // Setup some visualization
    
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( x,"position x" );
    window1.addSubplot( y,"position y" );
    window1.addSubplot( z,"position z" );
    window1.addSubplot( u,"verlocity x" );
    window1.addSubplot( v,"verlocity y" );
    window1.addSubplot( w,"verlocity z" );

    GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
    window3.addSubplot( T_r,"rotation-acc x" );
    window3.addSubplot( T_l,"rotation-acc y" );
    window3.addSubplot( T_u,"rotation-acc z" );

    

    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    algorithm << window3;
    algorithm.solve();
  }
  else
  {
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("vide_ray_ccode") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}
