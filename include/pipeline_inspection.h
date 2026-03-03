#pragma once

#include <mutex>
#include <queue>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include <gazebo_msgs/ModelStates.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "visual_servo.h"
#include "mpc_ros_application.h"
#include "mpc_config.h"
#include "disturbance_observer.h"
#include "plot.h"

class PipelineInspection: public MpcRosApplication 
{
private:
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gdt_subscriber_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_subscriber_;
    image_transport::Publisher edge_detection_publisher_;


    std::mutex rgb_mutex_;
    std::queue<sensor_msgs::ImageConstPtr> rgb_buf;
    std::thread edge_detection_thread_;
    std::thread trajectory_creation_thread_;
    std::thread image_show_thread_;

    static constexpr double kPipeDiameter = 1.5;  

    real_t target_roll_ = 0.0;
    real_t target_pitch_ = 0.0;
    real_t target_yaw_ = 0.0;

    float current_s_rho_;
    float current_s_theta_;
    float current_pos_x_;
    float current_pos_y_;
    float current_pos_z_;
    Eigen::Quaterniond current_att_;

    float current_bodyvel_x_;
    float current_vel_x_;
    float current_vel_y_;  
    float current_vel_z_;
    float current_wx_;
    float current_wy_;
    float current_wz_;
    Eigen::Matrix<float, 6, 1> nompc_reference_controls_;
    Eigen::Vector<real_t, ACADO_NX> real_states_ = Eigen::Vector<real_t, ACADO_NX>::Zero();
    Eigen::Vector<real_t, ACADO_NX> noise_states_ = Eigen::Vector<real_t, ACADO_NX>::Zero();
    Eigen::Vector<real_t, ACADO_NU> real_control_ = Eigen::Vector<real_t, ACADO_NU>::Zero();
    Eigen::Vector<real_t, ACADO_NU> delta_control_ = Eigen::Vector<real_t, ACADO_NU>::Zero();
    double vh_;
    
   
    int roi_height_;
    double image_height_;
    double image_width_;
    cv::Mat result_window_;
    cv::Rect roi;
    
    Eigen::Matrix<real_t, 2, 6> jacobian_;
    Eigen::Matrix<double, 2, 6> jacobian_d_;
    Eigen::Vector<real_t, 3> rpy_{0.0, 0.0, 0.0};
    Eigen::Matrix<real_t, 6, 1> vw_;

    // for constant trajectory
    const float delta_theta_ = 0.005/3;
    const float delta_t_ = 0.01;
    const float speed_ = 0.5;

    int half_frequency_ = 1;
    bool is_first_estimation_ = true;

    // double current_time_ = 0;

    // Eigen::Matrix<real_t, ACADO_NU, 1> hover_control_{kG, 0, 0, 0};
    Eigen::Matrix<real_t, 3, 1> start_point_{0.0, 0.0, 0.0};
    Eigen::Matrix<real_t, 3, 1> end_point_{0.0, 0.0, 0.0};

    double cruise_speed_ = 0.5;  // m/s

    Eigen::VectorX<real_t> d_hat_ = Eigen::VectorX<real_t>::Zero(5);
    Eigen::VectorX<real_t> z_hat_ = Eigen::VectorX<real_t>::Zero(8);
    Eigen::VectorX<real_t> real_d_hat_ = Eigen::VectorX<real_t>::Zero(8);
    Eigen::VectorX<real_t> pre_d_hat_ = Eigen::VectorX<real_t>::Zero(8);
    Eigen::VectorX<real_t> d_d_hat_ = Eigen::VectorX<real_t>::Zero(8);

    enum ControlMode{
        FIXED_TRAJECTORY,
        VISUAL_SERVO,
    };

    enum PosStatus{
        START_HOVER,
        TRAJECTORY1,
        TRAJECTORY2,
        TRAJECTORY3,
        TRAJECTORY4,
        TRAJECTORY5,
        VISUAL_SERVOING,
        END_HOVER,
    };

    bool image_state_ = false;
    real_t predicted_s_rho_ = 0.0;
    real_t predicted_s_theta_ = M_PI/2.0;

    Eigen::Matrix<real_t, 2, 1> predicted_s_ = Eigen::Matrix<real_t, 2, 1>::Zero();

    struct CurrentTargetStates
    {
        real_t target_s_rho;
        real_t target_s_theta;
        real_t target_x;
        real_t target_y;
        real_t target_z;
        real_t target_vx;
        real_t target_vy;
        real_t target_vz;
        real_t target_vb_x;
        real_t target_vb_y;
        real_t target_vb_z;
        Eigen::Quaternion<real_t> target_q;
        real_t target_roll;
        real_t target_pitch;
        real_t target_yaw;
        real_t target_wx;
        real_t target_wy;
        real_t target_wz;
        real_t target_torque_x;
        real_t target_torque_y;
        real_t target_torque_z;
        real_t target_thrust;
        
        CurrentTargetStates(): target_s_rho(0.0), target_s_theta(M_PI/2.0),
                               target_x(0.0), target_y(0.0), target_z(0.0),
                               target_vx(0.0), target_vy(0.0), target_vz(0.0), 
                               target_vb_x(0.0), target_vb_y(0.0), target_vb_z(0.0),
                               target_wx(0.0), target_wy(0.0), target_wz(0.0), target_thrust(kG*kMass),
                               target_torque_x(0.0), target_torque_y(0.0), target_torque_z(0.0),
                               target_roll(0.0), target_pitch(0.0), target_yaw(0.0)
        {
            target_q.w() = 1.0;
            target_q.x() = 0.0;
            target_q.y() = 0.0;
            target_q.z() = 0.0;
        }
    };

    struct MpcConstraints
    {
        real_t max_roll;
        real_t max_wx;

        MpcConstraints(): max_roll(8.0/180.0*M_PI), max_wx(0.2)
        {
        }
    } ;
    
    PosStatus pos_status_;
    ControlMode control_mode_;   
    CurrentTargetStates current_target_states_;
    VisualServoing visual_servoing_;
    DisturbanceObserver disturbance_observer_{0.01};
    MpcConstraints mpc_constraints_;
    DataLogger logger_{"/home/lw/data/ekf/data.csv"};
    DataLogger logger_disturbance_{"/home/lw/data/ekf/disturbance.csv"};
    DataLogger logger_velocity_{"/home/lw/data/ekf/velocity.csv"};
    DataLogger logger_kalman_gain_{"/home/lw/data/ekf/kalman_gain.csv"};
    DataLogger logger_error_{"/home/lw/data/ekf/estimation_error.csv"};
    DataLogger logger_control_{"/home/lw/data/ekf/delta_control.csv"};
    DataLogger logger_image_features_{"/home/lw/data/ekf/image_features.csv"};
    DataLogger logger_real_image_features_{"/home/lw/data/ekf/real_image_features.csv"};

public:
    PipelineInspection();
    ~PipelineInspection();

    bool let_mpc_run_;

    void init() override;
    void setReference() override;
    void setOnlineData() override;
    void mpcProcess() override;
    void gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& img_msg);
    std::pair<cv::Mat, cv::Mat> imageConversion(const sensor_msgs::ImageConstPtr& msg);
    void createTrajectory();
    double addWhiteNoise(double data, double mean, double stddev, unsigned int seed = 0);
    void setStartandEndPoint(const Eigen::Matrix<real_t, 3, 1>& start_point, 
                             const Eigen::Matrix<real_t, 3, 1>& end_point);
    Eigen::Quaternionf euler2Quaternion(const real_t& roll, const real_t& pitch, const real_t& yaw);
    Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q);
    Eigen::Vector3f getTargetPosition();
    void calPredictedFeatures(double dt);
    Eigen::Matrix<double, 6, 6> getTransformMatrix();
    bool isClosetoEndPoint();
    void dataLog(double time);
    Eigen::Matrix<real_t, 6, 1> getVelocityWorld()
    {
        return vw_;
    }
    void imageShow();
};

class PipelineInspectionFactory: public MpcFactory 
{
public:
    MpcRosApplication* createMpcRosApplication() override
    {
        return new PipelineInspection();
    }
};
