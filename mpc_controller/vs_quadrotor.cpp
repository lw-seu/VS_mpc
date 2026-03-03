#include <memory>
#include <math.h>
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

int main()
{
    USING_NAMESPACE_ACADO

    const bool CODE_GEN = true;

    // Variables 
    DifferentialState     s_rho, s_theta;
    DifferentialState     v_x, v_y, v_z;
    DifferentialState     roll, pitch, yaw;
    Control               w_x, w_y, w_z, T;
    OnlineData            j_00, j_01, j_02, j_03, j_04, j_05;
    OnlineData            j_10, j_11, j_12, j_13, j_14, j_15;
    OnlineData            d_rho, d_theta, d_x, d_y, d_z;
    // Parameters with exemplary values. These are set/overwritten at runtime.
    const double t_start = 0.0;     // Initial time [s]
    const double t_end = 2;       // Time horizon [s]
    const double dt = 0.1;          // Discretization time [s]
    const int N = round(t_end/dt);  // Number of nodes
    const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
    const double m = 1.5;           // iris mass [kg]

    // Model equations
    DifferentialEquation f; 
    f << dot(s_rho) == j_00 * v_x + j_01 * v_y + j_02 * v_z + j_03 * w_x + j_04 * w_y + j_05 * w_z + d_rho;
    // f << dot(s_rho) == j_00 * v_x + j_01 * v_y + j_02 * v_z + j_03 * w_x + d_rho;
    f << dot(s_theta) == j_10 * v_x + j_11 * v_y + j_12 * v_z + j_13 * w_x + j_14 * w_y + j_15 * w_z + d_theta;
    // f << dot(s_theta) == j_15 * w_z + d_theta;
    f << dot(v_x) ==  ( cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * T/m + d_x;
    f << dot(v_y) ==  ( sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) * T/m + d_y;
    f << dot(v_z) ==  ( cos(pitch) * cos(roll) ) * T/m - g_z + d_z;
    f << dot(roll) ==  w_x + sin(roll) * tan(pitch) * w_y + cos(roll) * tan(pitch) * w_z ;
    f << dot(pitch) ==  cos(roll) * w_y - sin(roll) * w_z;
    f << dot(yaw) ==  sin(roll) / cos(pitch) * w_y + cos(roll) / cos(pitch) * w_z;



    // Reference functions and weighting matrices
    Function h, hN;
    IntermediateState v_b_x;

    v_b_x = cos(yaw) * v_x + sin(yaw)  * v_y;

    h << s_rho << s_theta 
    << v_b_x << v_z 
    << w_x << w_y << w_z << T;  

    // End cost vector consists of all states (no inputs at last state).
    hN << s_rho << s_theta 
    << v_b_x << v_z;

    DMatrix Q(h.getDim(), h.getDim());
    Q.setIdentity();
    Q(0,0) = 100;   // s_rho
    Q(1,1) = 100;   // s_pitch
    Q(2,2) = 10;   // v_x
    Q(3,3) = 10;   // v_y
    Q(4,4) = 10;   // v_z
    Q(5,5) = 1;   // roll
    Q(6,6) = 100;   // pitch
    Q(7,7) = 100;   // yaw
    // Q(8,8) = 1;   // vb_x
    // Q(9,9) = 10;   // vb_y
    // Q(10,10) = 10;   // vb_z
    // Q(11,11) = 10;   // wx


    // End cost weight matrix
    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0,0) = Q(0,0);   // s_rho
    QN(1,1) = Q(1,1);   // s_pitch
    QN(2,2) = Q(2,2);   // v_x
    QN(3,3) = Q(3,3);   // v_y
    // QN(4,4) = Q(4,4);   // v_z
    // QN(5,5) = Q(5,5);   // roll
    // QN(6,6) = Q(6,6);   // pitch
    // QN(7,7) = Q(7,7);   // yaw


    // Set a reference for the analysis (if CODE_GEN is false).
    // Reference is at x = 2.0m in hover (qw = 1).
    DVector r(h.getDim());    // Running cost reference
    r.setZero();

    DVector rN(hN.getDim());   // End cost reference
    rN.setZero();


    OCP ocp( t_start, t_end, N );


    if(!CODE_GEN)
    {
        // For analysis, set references.
        ocp.minimizeLSQ( Q, h, r );
        ocp.minimizeLSQEndTerm( QN, hN, rN );
    }else{
        // For code generation, references are set during run time.
        BMatrix Q_sparse(h.getDim(), h.getDim());
        Q_sparse.setIdentity();

        BMatrix QN_sparse(hN.getDim(), hN.getDim());
        QN_sparse.setIdentity();

        ocp.minimizeLSQ( Q_sparse, h);
        ocp.minimizeLSQEndTerm( QN_sparse, hN );
    }

    // Add system dynamics
    ocp.subjectTo( f );


    ocp.setNOD(17);

    if(!CODE_GEN)
    {
        // Set initial state
        ocp.subjectTo( AT_START, v_x ==  0.0 );
        ocp.subjectTo( AT_START, v_y ==  0.0 );
        ocp.subjectTo( AT_START, v_z ==  0.0 );
        ocp.subjectTo( AT_START, w_x ==  0.0 );
        ocp.subjectTo( AT_START, w_y ==  0.0 );
        ocp.subjectTo( AT_START, w_z ==  0.0 );

        // Setup some visualization
        GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
        window1.addSubplot( v_x,"verlocity x" );
        window1.addSubplot( v_y,"verlocity y" );
        window1.addSubplot( v_z,"verlocity z" );

        GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
        window3.addSubplot( w_x,"rotation-acc x" );
        window3.addSubplot( w_y,"rotation-acc y" );
        window3.addSubplot( w_z,"rotation-acc z" ); 
        // window3.addSubplot( T,"Thrust" );


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
        mpc.set( GENERATE_TEST_FILE,          YES);
        mpc.set( GENERATE_MAKE_FILE,          NO);
        mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
        mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

        // Finally, export everything.
        if(mpc.exportCode("quadrotor_mpc_export") != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );
        mpc.printDimensionsQP( );
    }
    
    return EXIT_SUCCESS;
}