#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <cv_bridge/cv_bridge.h>
#include "pipeline_inspection.h"

PipelineInspection::PipelineInspection(): it_(nh_),
                   pos_status_(PosStatus::START_HOVER),
                   control_mode_(ControlMode::VISUAL_SERVO),
                   let_mpc_run_(false)
{
    init();
}

PipelineInspection::~PipelineInspection()
{
    cv::destroyAllWindows();
}

void PipelineInspection::init()
{
    Eigen::Matrix<real_t, ACADO_NX, 1> initial_states;
    predicted_s_ = {0.0, M_PI_2};

    initial_states = {0.0, M_PI/2,
                      0.0, 0.0, 0.0, 
                      0.0, 0.0, 0.0};
    q_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() <<
        0.3,                                        // s_rho
        0.3,                                        // s_theta
        0.01,                                          // v_h
        1                                         // v_z
        ).finished().asDiagonal();                 

    r_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() <<
        0.05,                             // wx     fixed
        0.05,                             // wy
        0.01,                              // wz
        0.0).finished().asDiagonal();      // T

    current_states_ = initial_states;
    predicted_states_ = initial_states.replicate(1, ACADO_N + 1);
    reference_states_.setZero();
    reference_controls_.setZero();
    current_control_.setZero();


    mpc_wrapper_.initMpc(current_states_, predicted_states_, reference_states_, 
        reference_controls_, q_, r_);

    mpc_wrapper_.getMpcBound(control_lower_bound_, control_upper_bound_);  

    setStartandEndPoint(Eigen::Vector<real_t, 3>{2.0, 1.0, 2.5},
                        Eigen::Vector<real_t, 3>{21.5+3*sin(M_PI/6)+13*cos(M_PI/6)+sqrt(2)*cos(M_PI/6), 
                                        1-(3-3*cos(M_PI/6))-13*sin(M_PI/6)-sqrt(2)*sin(M_PI/6), 
                                        2.5+4*powf(sin(M_PI_4/2),2)+14*sin(M_PI_4)});
    current_target_states_.target_x = start_point_[0];
    current_target_states_.target_y = start_point_[1];
    current_target_states_.target_z = start_point_[2];

    //opencv
    roi_height_ = visual_servoing_.getRoiHeight();
    image_height_ = visual_servoing_.getImageHeight();
    image_width_ = visual_servoing_.getImageWidth();
    result_window_ = visual_servoing_.getResultWindow();
    
    gdt_subscriber_ = nh_.subscribe("/gazebo/model_states",
                1, &PipelineInspection::gdtCallback, this);
    edge_detection_publisher_ = it_.advertise("/edge_detection", 1);
    depth_subscriber_ = it_.subscribe("/iris/camera/depth/image_raw", 1, 
                &PipelineInspection::depthCallback, this);

    trajectory_creation_thread_ = std::thread(&PipelineInspection::createTrajectory, this);
    #if IS_VISUAL_PREDICTION_ON
        image_show_thread_ = std::thread(&PipelineInspection::imageShow, this);
    #endif
}


void PipelineInspection::depthCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    // rgb_mutex_.lock();
    // rgb_buf.push(img_msg);
    // rgb_mutex_.unlock();
    image_state_= true;
    std::pair<cv::Mat, cv::Mat> src_dst = imageConversion(img_msg);
    if(control_mode_ == FIXED_TRAJECTORY && 
        pos_status_ != PosStatus::START_HOVER && pos_status_ != PosStatus::END_HOVER)
    {
        visual_servoing_.imageProcess(src_dst.second);
        // visual_servoing_.imageProcess(src_dst.second);
        // visual_servoing_.calculateJacobian();
    }
    else if(control_mode_ == VISUAL_SERVO && pos_status_ != PosStatus::END_HOVER)
    {       
        visual_servoing_.visualServoing(src_dst.second);  

        if(pos_status_ == VISUAL_SERVOING)
        {
            static auto start_time = std::chrono::steady_clock::now();
            std::pair<double, double> image_features = visual_servoing_.getStates();
            current_s_rho_ = static_cast<real_t>(image_features.first);
            current_s_theta_ = static_cast<real_t>(image_features.second);   
            Eigen::VectorXd real_image_features(2);
            real_image_features << current_s_rho_, current_s_theta_-M_PI_2;
            auto current_time = std::chrono::steady_clock::now();
            logger_real_image_features_.logData(std::chrono::duration<double>(current_time - start_time).count(), 
                                    current_pos_x_, real_image_features); 
        }
        
    }
    else if(pos_status_ == END_HOVER)
    {
        cv::destroyAllWindows();
    }
}

void PipelineInspection::imageShow()
{
    while(ros::ok())
    {
        Eigen::Vector<cv::Point2d, 2> m_line;
        m_line << cv::Point2d(static_cast<double>(image_width_/2.0 + current_states_(0)  * 454.68 -
                                static_cast<double>(roi_height_)/(2.0*tan(current_states_(1)))), 
                                image_height_/2.0 - static_cast<double>(roi_height_)/2.0),
                cv::Point2d(static_cast<double>(image_width_/2.0 + current_states_(0) * 454.68 +
                                static_cast<double>(roi_height_)/(2.0*tan(current_states_(1))) ), 
                                image_height_/2.0 + static_cast<double>(roi_height_)/2.0);
        cv::line(result_window_, m_line[0], m_line[1], cv::Scalar(255, 0, 0), 2);
        cv::imshow("result", result_window_);
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
}

Eigen::Vector3d PipelineInspection::Quaternion2Euler(Eigen::Quaterniond q)
{
    Eigen::Vector3d euler(0, 0, 0);
    euler(0) = atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                    q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    euler(1) = asin(-2 * (q.x() * q.z() - q.w() * q.y()));
    euler(2) = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return euler;
}

double PipelineInspection::addWhiteNoise(double data, double mean, double stddev, unsigned int seed)
{
    static std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(mean, stddev);
    return data + distribution(generator);
}

void PipelineInspection::gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg)
{ 
    // ROS_INFO("Recive odom data");
    // Eigen::Matrix<real_t, ACADO_NX, 1> odom;
    // state_mutex.lock();
    int i = 0;

    std::pair<double, double> image_features = visual_servoing_.getStates();
    current_states_[i++] = static_cast<real_t>(image_features.first);
    current_states_[i++] = static_cast<real_t>(image_features.second);


    // printf("s_rho: %f\t", current_states_(0));

    for(int j = 0; j < gdt_msg->name.size(); j++)
    {
        if(gdt_msg->name[j] == "iris")
        {
            current_pos_x_ = static_cast<real_t>(gdt_msg->pose[j].position.x);
            current_pos_y_ = static_cast<real_t>(gdt_msg->pose[j].position.y);
            current_pos_z_ = static_cast<real_t>(gdt_msg->pose[j].position.z);
            current_att_.w() = static_cast<double>(gdt_msg->pose[j].orientation.w);
            current_att_.x() = static_cast<double>(gdt_msg->pose[j].orientation.x);
            current_att_.y() = static_cast<double>(gdt_msg->pose[j].orientation.y);
            current_att_.z() = static_cast<double>(gdt_msg->pose[j].orientation.z);
            current_att_.normalize();
            current_vel_x_ = static_cast<real_t>(gdt_msg->twist[j].linear.x);
            current_vel_y_ = static_cast<real_t>(gdt_msg->twist[j].linear.y);
            current_vel_z_ = static_cast<real_t>(gdt_msg->twist[j].linear.z);
            current_wx_ = static_cast<real_t>(gdt_msg->twist[j].angular.x);
            current_wy_ = static_cast<real_t>(gdt_msg->twist[j].angular.y);
            current_wz_ = static_cast<real_t>(gdt_msg->twist[j].angular.z);
            break;
        }
    }
    
    Eigen::Vector<real_t, 3> rpy = Quaternion2Euler(current_att_).cast<real_t>();
    vh_ = cos(rpy(2)) * current_vel_x_ + sin(rpy(2)) * current_vel_y_; 

#if SYSTEM_CONTROL == BODYRATE_THRUST_CONTROL
    current_states_[i++] = current_vel_x_;
    current_states_[i++] = current_vel_y_;
    current_states_[i++] = current_vel_z_;
    current_states_[i++] = rpy(0);
    current_states_[i++] = rpy(1);
    current_states_[i++] = rpy(2);
    // current_states_[0] -= tan(rpy(0)); // + addWhiteNoise(0, 0, 0.01);
    // printf("s_rho: %f\n", current_states_[0]);
    // printf("roll: %f\n, pitch: %f\n", rpy(0),rpy(1));
    real_states_ = current_states_;
    current_states_[2] = addWhiteNoise(current_states_[2], 0, 0.01);
    current_states_[3] = addWhiteNoise(current_states_[3], 0, 0.01);
    current_states_[4] = addWhiteNoise(current_states_[4], 0, 0.01);
    current_states_[5] = addWhiteNoise(current_states_[5], 0, 0.01);
    current_states_[6] = addWhiteNoise(current_states_[6], 0, 0.01);
    current_states_[7] = addWhiteNoise(current_states_[7], 0, 0.01);
    noise_states_ = current_states_;
#endif 
 
    if(pos_status_ == VISUAL_SERVOING)
    {
        static auto start_time = std::chrono::steady_clock::now();
            auto current_time = std::chrono::steady_clock::now();
        dataLog(std::chrono::duration<double>(current_time - start_time).count());
    }
    is_new_data_ = true;
    mpc_count_ = 0;
    
    // state_mutex.unlock();
}

void PipelineInspection::calPredictedFeatures(double dt)
{
    Eigen::Matrix<real_t, 6, 1> all_velocity;
    all_velocity << current_vel_x_, current_vel_y_, current_vel_z_,
                    current_wx_, current_wy_, current_wz_;
    predicted_s_ = predicted_s_ + jacobian_ * all_velocity * dt; 
}

void PipelineInspection::mpcProcess()
{
    while (ros::ok())
    {
        if(is_new_data_)
        {
            is_new_data_ = false;

            Eigen::Matrix<double, 6, 6> adj_twb = Eigen::Matrix<double, 6, 6>::Zero();
            adj_twb = getTransformMatrix();
            jacobian_ = (visual_servoing_.getJacobian()*adj_twb.inverse()).cast<real_t>();

            setReference();
            setParameter();

            Eigen::VectorXd y = Eigen::VectorXd::Zero(5);
            #if SYSTEM_CONTROL == VELOCITY_CONTROL
                y << current_states_(0), current_states_(1), 0.0, 0.0, 0.0;
            #else
                y << current_states_(0), current_states_(1), current_states_(2), current_states_(3), current_states_(4);
            #endif

            
            static auto pre_time = std::chrono::steady_clock::now();
            static auto current_time = std::chrono::steady_clock::now();    
            if(is_first_estimation_)
            {
                is_first_estimation_ = false;
                Eigen::VectorXd z = Eigen::VectorXd::Zero(5);
                z << y; 
                disturbance_observer_.setFirstZ(z);
            }
            else
            {   
                current_time = std::chrono::steady_clock::now();
                disturbance_observer_.setDt(std::chrono::duration<double>(current_time - pre_time).count());
            }

            Eigen::VectorXd u = Eigen::VectorXd::Zero(ACADO_NU);
            u << current_control_(0), current_control_(1), current_control_(2), current_control_(3);
            jacobian_d_ << jacobian_.cast<double>();
            if(image_state_)
            {
                predicted_s_ << y(0), y(1);
            }
            else
            {
                calPredictedFeatures(std::chrono::duration<double>(current_time - pre_time).count());
                y(0) = predicted_s_(0);
                y(1) = predicted_s_(1);
            }

            pre_time = current_time;

            if(image_state_)
            {
                disturbance_observer_.eskf_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, jacobian_d_);
            }
            else
            {
                disturbance_observer_.eskf_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, jacobian_d_, 1);
            }

            d_hat_.block<5,1>(0,0) = disturbance_observer_.getDisturbance().block<5, 1>(0, 0).cast<real_t>();
            z_hat_.block<5,1>(0,0) = disturbance_observer_.getState().block<5, 1>(0, 0).cast<real_t>();
            current_states_(0) = z_hat_(0);
            current_states_(1) = z_hat_(1);
            current_states_(2) = z_hat_(2);
            current_states_(3) = z_hat_(3);
            current_states_(4) = z_hat_(4);     
            // std::cout << "d_hat: " << d_hat_.transpose() << "\tdt: "<< current_time_ - pre_time << std::endl;

            setOnlineData();

            
            image_state_ = false;

            // current_time = ros::Time::now().toNSec()/1000;
            if(mpc_wrapper_.solveMpc(current_states_))
            {
                // dt = ros::Time::now().toNSec()/1000 - current_time;
                // printf("MPC solve time: %f\n", dt);          
                mpc_wrapper_.getMpcControl(current_control_, 0);
                // std::cout << "mpc_control:" << current_control_.transpose() << std::endl;
            }
            else
            {
                ROS_WARN("MPC solve failed!");
            }
            
        }
        else
        {
            mpc_count_++;
            if(mpc_count_ > 100)
            {
                mpc_error_ = DATA_LOST;
            }
        } 
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
void PipelineInspection::setReference()
{
    Eigen::Matrix<real_t, 6, 1> null_velocity;
    double vh;

    null_velocity =  visual_servoing_.getNullVelocity().cast<real_t>();
    current_target_states_.target_vb_x = null_velocity(0);
    current_target_states_.target_vb_y = null_velocity(1);
    current_target_states_.target_vb_z = null_velocity(2);
    null_velocity = getTransformMatrix().cast<real_t>() * null_velocity;
    current_target_states_.target_vx = null_velocity(0);
    current_target_states_.target_vy = null_velocity(1);
    current_target_states_.target_vz = null_velocity(2);
    current_target_states_.target_wx = null_velocity(3);
    current_target_states_.target_wy = null_velocity(4);
    current_target_states_.target_wz = null_velocity(5); 
    // printf("%f\n", sqrt(pow(null_velocity(0), 2) + pow(null_velocity(1), 2)));
    // std::cout << "null_velocity: " << null_velocity.transpose() << std::endl;

    reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
        current_target_states_.target_s_rho, current_target_states_.target_s_theta, 
        current_target_states_.target_vb_x, current_target_states_.target_vb_z
        ).finished().replicate(1, ACADO_N + 1);

    reference_controls_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() <<
        current_target_states_.target_wx, current_target_states_.target_wy, 
        current_target_states_.target_wz, 
        current_target_states_.target_thrust).finished().replicate(1, ACADO_N);
    
    // std::cout << "reference_controls_" << reference_controls_.col(0).transpose() << std::endl;
    // std::cout << "reference_state" << reference_states_.col(0).transpose() << std::endl;
    mpc_wrapper_.setMpcReference(reference_states_, reference_controls_);
}


Eigen::Matrix<double, 6, 6> PipelineInspection::getTransformMatrix()
{
    Eigen::Matrix<double, 6, 6> adj_twb = Eigen::Matrix<double, 6, 6>::Zero();

    double w_q = static_cast<double> (current_att_.w());
    double x_q = static_cast<double> (current_att_.x());
    double y_q = static_cast<double> (current_att_.y());
    double z_q = static_cast<double> (current_att_.z());
    
    Eigen::Quaterniond quaternion(w_q, x_q, y_q, z_q); // w, x, y, z

    Eigen::Matrix3d rwb = quaternion.toRotationMatrix();
    rpy_ = Quaternion2Euler(quaternion).cast<real_t>();
    
    adj_twb.block<3, 3>(0, 0) = rwb;
    adj_twb.block<3, 3>(3, 3) << 1, 0, 0,
                                 0, 1, 0,
                                 0, 0, 1;

    return adj_twb;
}

void PipelineInspection::setOnlineData()
{
    Eigen::Matrix<real_t, ACADO_NOD, 1> online_data;
    
    Eigen::Matrix<real_t, 2, 3> L;
    online_data << jacobian_.row(0).transpose(), jacobian_.row(1).transpose(), d_hat_;

    mpc_wrapper_.setMpcOnlineData(online_data.replicate(1, ACADO_N + 1));
}
std::pair<cv::Mat, cv::Mat> PipelineInspection::imageConversion(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    cv::Mat norm;

    img = cv_bridge::toCvShare(msg, "32FC1")->image;
    cv::normalize(img, norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    return std::make_pair(img, norm);
}

Eigen::Quaternionf PipelineInspection::euler2Quaternion(const real_t& roll, const real_t& pitch, const real_t& yaw)
{
    real_t r, p, y;
    r = roll/2.0;
    p = pitch/2.0;
    y = yaw/2.0;
    current_target_states_.target_q.w() = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);
    current_target_states_.target_q.x() = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);
    current_target_states_.target_q.y() = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);
    current_target_states_.target_q.z() = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);
    current_target_states_.target_q.normalize();

    return current_target_states_.target_q;
}

void PipelineInspection::createTrajectory()
{
    while(ros::ok())
    {
        switch (pos_status_)
        {
            case START_HOVER:
            {
                float horizontal_distance = 0.0;
                float z_distance = 0.0;
                horizontal_distance = sqrtf(powf(current_pos_x_- start_point_(0), 2) + 
                                        powf(current_pos_y_ - start_point_(1), 2));
                z_distance = current_pos_z_;
                if(horizontal_distance < 0.1 && z_distance > start_point_(2) - 0.2)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                    if(control_mode_ == FIXED_TRAJECTORY)
                    {
                        pos_status_ = TRAJECTORY1;
                    }
                    else if(control_mode_ == VISUAL_SERVO)
                    {
                        #if IS_USE_MPC
                            let_mpc_run_ = true;
                        #endif
                        pos_status_ = VISUAL_SERVOING;
                    }
                        
                }
                break;
            }

            case VISUAL_SERVOING:
            {
                if(visual_servoing_.getFrontWidth() == 0.0)   // 设置终点位置
                {
                    pos_status_ = END_HOVER;
                    let_mpc_run_ = false;
                }
                break;
            }

            case END_HOVER:
            {
                current_target_states_.target_z = current_pos_x_;
                current_target_states_.target_x = current_pos_y_;
                current_target_states_.target_y = current_pos_z_;
                current_target_states_.target_vx = 0.0;
                current_target_states_.target_vy = 0.0;
                current_target_states_.target_vz = 0.0;
                break;
            }
            default:
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PipelineInspection::setStartandEndPoint(
    const Eigen::Matrix<real_t, 3, 1>& start_point, 
    const Eigen::Matrix<real_t, 3, 1>& end_point)
{
    start_point_ = start_point;
    end_point_ = end_point;
}

Eigen::Vector3f PipelineInspection::getTargetPosition()
{
    Eigen::Matrix<real_t, 3, 1> target_pos;

    target_pos << current_target_states_.target_x,
                   current_target_states_.target_y,
                   current_target_states_.target_z;
    
    return target_pos;
}

bool PipelineInspection::isClosetoEndPoint()
{
    float distance;
    distance = std::abs(current_pos_x_-81.5);

    return distance <= 0.01;
}

void PipelineInspection::dataLog(double time)
{

    Eigen::VectorXd evaluation(4);
    evaluation << current_s_rho_, current_s_theta_-M_PI_2, vh_, visual_servoing_.getD();
    logger_.logData(time, current_pos_x_, evaluation);

    Eigen::VectorXd image_features(6);
    image_features << current_s_rho_, predicted_s_(0), z_hat_(0), current_s_theta_, predicted_s_(1), z_hat_(1);
    logger_image_features_.logData(time, current_pos_x_, image_features);

    Eigen::VectorXd d = disturbance_observer_.getDisturbance();
    logger_disturbance_.logData(time, current_pos_x_, d);

    Eigen::VectorXd estimeated = disturbance_observer_.getState();
    Eigen::VectorXd s(9);
    s << noise_states_(2), estimeated(2), real_states_(2), noise_states_(3), estimeated(3), 
        real_states_(3), noise_states_(4), estimeated(4), real_states_(4);
    logger_velocity_.logData(time, current_pos_x_, s);

    Eigen::VectorXd error(3);
    error << estimeated(2)-real_states_(2), estimeated(3)-real_states_(3), estimeated(4)-real_states_(4);
    logger_error_.logData(time, current_pos_x_, error);
}