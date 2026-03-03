#include <stdio.h>
#include <iostream>

#include "mpc_wrapper.h"

#include <acado/acado_gnuplot.hpp>
#include <acado/curve/curve.hpp>
#include <acado/acado_toolkit.hpp>


MpcWrapper::MpcWrapper()
{
    
}

void MpcWrapper::initMpc(
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, 1>> &current_states,
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, ACADO_N + 1>> &predicted_states,
    const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1>> &reference_states,
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_N>> &reference_inputs,
    const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>>& Q,
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>>& R)
{
    // Clear solver memory.
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));
  
    // Initialize the solver.
    acado_initializeSolver();

    acado_current_states_ = current_states;
    acado_predicted_states_ = predicted_states;

    acado_controls_.setZero();
    acado_reference_.block(0, 0, kCostStateSize, ACADO_N) = 
        reference_states.block(0, 0, kCostStateSize, ACADO_N);

    acado_reference_.block(kCostStateSize, 0, ACADO_NU, ACADO_N) = reference_inputs;
    acado_terminal_reference_ = reference_states.rightCols(1);

    setMpcWeight(Q, R);
    
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
}

bool MpcWrapper::setMpcReference(
    const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1>>& ref_states,
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_N>>& ref_control)
{
    if (ref_states.rows() != kCostStateSize || ref_states.cols() != ACADO_N + 1 
        || ref_control.cols() != ACADO_N || ref_control.rows() != ACADO_NU  )
    {
        printf("MpcWrapper::setMpcReference:");
        if(ref_states.rows() != kCostStateSize)
        {
            printf("wrong row size of state reference\n");
        }
        
        if(ref_states.cols() != ACADO_N + 1)
        {
            printf("wrong column size of state reference\n");
        }
        if(ref_control.rows() != ACADO_NU)
        {
            printf("wrong row size of control reference\n");
        }
            
        if (ref_control.cols() != ACADO_N)
        {
            printf("wrong column size of control reference\n");
        }

        return false;
    }
    acado_reference_.block(0, 0, kCostStateSize, ACADO_N) = ref_states.leftCols(ACADO_N);
    acado_reference_.block(kCostStateSize, 0, ACADO_NU, ACADO_N) = ref_control;
    acado_terminal_reference_ = ref_states.rightCols(1);
    
    acado_initializeNodesByForwardSimulation();
    return true;
}

bool MpcWrapper::setMpcWeight(
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>>& Q,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>>& R,
        real_t state_cost_scaling, real_t input_cost_scaling)
{
    Eigen::Matrix<bool, kCostStateSize, kCostStateSize> mask1 = Q.array() < 0;
    Eigen::Matrix<bool, ACADO_NU, ACADO_NU> mask2 = R.array() < 0;
    if(mask1.any() || mask2.any())
    {
        if(mask1.any())
        {
            printf("MpcWrapper::setMpcWeight: States weight matrix Q should be non-negative!\n");
        }
        if(mask2.any())
        {
            printf("MpcWrapper::setMpcWeight: Inputs weight matrix R should be non-negative!\n");
        }
        return false;
    }

    Eigen::Matrix<real_t, ACADO_NY, ACADO_NY> W_;
    Eigen::Matrix<real_t, ACADO_NYN, ACADO_NYN> WN_;

    W_.block(0, 0, kCostStateSize, kCostStateSize) = Q;
    W_.block(kCostStateSize, kCostStateSize, ACADO_NU, ACADO_NU) = R;
    WN_ = W_.block(0, 0, kCostStateSize, kCostStateSize);

    real_t state_scale{1.0};
    real_t input_scale{1.0};
    for(int i = 0; i < ACADO_N; i++)
    { 
        state_scale = exp(- real_t(i)/real_t(ACADO_N)
        * real_t(state_cost_scaling));
        input_scale = exp(- real_t(i)/real_t(ACADO_N)
        * real_t(input_cost_scaling));
        acado_weight_.block(0, i*ACADO_NY, kCostStateSize, kCostStateSize) =
        W_.block(0, 0, kCostStateSize, kCostStateSize) * state_scale;
        acado_weight_.block(kCostStateSize, i*ACADO_NY+kCostStateSize, ACADO_NU, ACADO_NU) =
        W_.block(kCostStateSize, kCostStateSize, ACADO_NU, ACADO_NU) * input_scale;
    } 
    acado_terminal_weight_ = WN_ * state_scale;

    return true;
}

bool MpcWrapper::setMpcOnlineData(
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>>& online_data)
{
    acado_online_data_ = online_data;
    return true;
}

bool MpcWrapper::setMpcEachOnlineData(
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NOD, 1>>& online_data, int index)
{
    acado_online_data_.col(index) = online_data;
    return true;
}

// bool MpcWrapper::setMpcAffineBounds(
//     const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>>& lower_affine_bounds,
//     const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>>& upper_affine_bounds)
// {
//     acado_lower_affine_bounds_ = lower_affine_bounds.replicate(1, ACADO_N);
//     acado_upper_affine_bounds_ = upper_affine_bounds.replicate(1, ACADO_N);
//     return true;
// }

bool MpcWrapper::solveMpc(
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, 1>>& current_states)
{
    if(current_states.size() != ACADO_NX)
    {
        printf("MpcWrapper::solveMpc: wrong size of current states!\n");
        return false;
    }

    acado_current_states_ = current_states;
    
    acado_feedbackStep();
    
    if(!acado_preparationStep())    
    {
        return true;
    }       
    else
    {
        printf("MpcWrapper::solveMpc: acado preparation failed!\n");
        return false;
    }
}

void MpcWrapper::getMpcControl(Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> control, int index)
{
    control = acado_controls_.col(index);
}

void MpcWrapper::getMpcControls(Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, ACADO_N>> controls)
{
    controls = acado_controls_;
}

void MpcWrapper::getMpcBound(
    Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> lower_bound,
    Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> upper_bound)
{
    lower_bound = acado_lower_bounds_.col(0);
    upper_bound = acado_upper_bounds_.col(0);
}