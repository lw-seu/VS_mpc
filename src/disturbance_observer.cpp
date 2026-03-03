#include <math.h>
#include <iostream>
#include "disturbance_observer.h"

DisturbanceObserver::DisturbanceObserver(double dt)
        : dt_(dt), d_(Eigen::VectorXd::Zero(8)), z_(Eigen::VectorXd::Zero(8))
{

}

void DisturbanceObserver::eskf_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob, const bool whichR)
{
    Eigen::MatrixXd A(eskf_.state_size_ + eskf_.extended_size_, eskf_.state_size_ + eskf_.extended_size_);
    A.setZero();
    A.block<2, 3>(0,2) = visual_jacob.leftCols(3);
    A.block<5,5>(0,5) = Eigen::MatrixXd::Identity(5, 5);

    Eigen::VectorXd G(eskf_.state_size_ + eskf_.extended_size_);
    G.setZero();
    G(4) = 1;


	Eigen::Vector3d R_WB_col2;
	R_WB_col2 << cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),
				 sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll),
				 cos(pitch) * cos(roll);
    // R_WB_col2 << sin(pitch) * cos(roll),
	// 			 - sin(roll),
	// 			 cos(pitch) * cos(roll);


    Eigen::MatrixXd B_u(eskf_.state_size_ + eskf_.extended_size_, eskf_.control_size_);
    B_u.setZero();
    B_u.block<2,3>(0,0) = visual_jacob.rightCols(3);
    B_u.block<3,1>(2, 3) = R_WB_col2 / m;


    A = Eigen::MatrixXd::Identity(eskf_.state_size_ + eskf_.extended_size_, eskf_.state_size_ + eskf_.extended_size_) + dt_ * A;
	B_u = dt_ * B_u;
	G = dt_ * G;

	Eigen::VectorXd tail =  - g_z * G;

    eskf_.set_Ak(A);

    if(whichR == 0)
        eskf_.set_Rk(eskf_.R0_);
    else
        eskf_.set_Rk(eskf_.R1_);

    eskf_.predict(Eigen::VectorXd::Zero(eskf_.extended_size_), tail, B_u, u);
    eskf_.update(y.block<5, 1>(0, 0));

    z_.block<5, 1>(0, 0) = eskf_.get_X_hat();
    d_.block<5, 1>(0, 0) = eskf_.get_F_hat();
}