#pragma once

#include <Eigen/Dense>

class ESKF {
public:
    ESKF(int state_dim, int extended_dim, int control_dim, int measurement_dim) {
        int total_dim = state_dim + extended_dim;

        X_hat_ = Eigen::VectorXd::Zero(state_dim);
        F_hat_ = Eigen::VectorXd::Zero(extended_dim);
        G_hat_ = Eigen::VectorXd::Zero(extended_dim);
        qk_ = Eigen::VectorXd::Ones(extended_dim) * 1e-8; 
        // qk_ = Eigen::VectorXd::Zero(extended_dim); 

        P_ = Eigen::MatrixXd::Identity(total_dim, total_dim) * 0.5;
        P_.block(state_dim, state_dim, extended_dim, extended_dim) = 1 * Eigen::MatrixXd::Identity(extended_dim, extended_dim);

        A_ = Eigen::MatrixXd::Identity(total_dim, total_dim); 
        B_ = Eigen::MatrixXd::Zero(total_dim, extended_dim);
        B_.block(state_dim, 0, extended_dim, extended_dim) = Eigen::MatrixXd::Identity(extended_dim, extended_dim);
        Bu_= Eigen::MatrixXd::Zero(total_dim, control_dim);
        C_ = Eigen::MatrixXd::Zero(measurement_dim, total_dim);
        C_.block(0, 0, measurement_dim, state_dim) = Eigen::MatrixXd::Identity(state_dim, state_dim);

        Qk_ = Eigen::MatrixXd::Zero(extended_dim, extended_dim);
        Q1_ = Eigen::MatrixXd::Zero(total_dim, total_dim);
        Q2_ = Eigen::MatrixXd::Zero(total_dim, total_dim);
        Q2_.block(0, 0, state_dim, state_dim) = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.001;
        Qk_ = extended_dim * qk_.asDiagonal();
        Q1_.block(state_dim, state_dim, extended_dim, extended_dim) = 4 * Qk_;
        R_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.01;
        R0_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.01;
        R1_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.1;

        compute_theta();

        std::cout << "theta: " << theta_ << std::endl;

        state_size_ = state_dim;
        measurement_size_ = measurement_dim;
        control_size_ = control_dim;
        extended_size_ = extended_dim;
    }

    void init(const Eigen::VectorXd& initial_state)
    {
        X_hat_ = initial_state;
    }
    Eigen::VectorXd get_X_hat() {
        return X_hat_;
    }

    Eigen::MatrixXd get_F_hat() {
        return F_hat_;
    }

    void set_Ak(const Eigen::MatrixXd& Ak) {
        A_ = Ak;
    }

    void set_Rk(const Eigen::MatrixXd& Rk) {
        R_ = Rk;
    }

    void set_qk(const Eigen::VectorXd& new_qk) {
        if (new_qk.size() == qk_.size()) {
            qk_ = new_qk;
        } else {
            std::cerr << "Error: qk size mismatch!" << std::endl;
        }
    }

    void compute_theta() {
        theta_ = std::sqrt(Q1_.trace() / P_.trace());
    }

    Eigen::VectorXd compute_G_hat(const Eigen::VectorXd& G_bar) {
        Eigen::VectorXd G_hat(G_bar.size());
        for (int i = 0; i < G_bar.size(); ++i) {
            double bound = std::sqrt(qk_(i));  // 直接使用已知 qk_
            G_hat(i) = std::max(std::min(G_bar(i), bound), -bound);
        }
        return G_hat;
    }

    Eigen::MatrixXd compute_Kk() {
        Eigen::MatrixXd temp = C_ * P_ * C_.transpose() + (1.0 / (1 + theta_)) * R_;
        return -A_ * P_ * C_.transpose() * temp.inverse();
    }

    void predict(const Eigen::VectorXd& G_bar, const Eigen::VectorXd tail, const Eigen::MatrixXd Bu, const Eigen::VectorXd& control_input) {
        G_hat_ = compute_G_hat(G_bar);
        Eigen::VectorXd extended_state(X_hat_.size() + F_hat_.size());
        extended_state << X_hat_, F_hat_;
        extended_state = A_ * extended_state + B_ * G_hat_ + Bu * control_input + tail;
        X_hat_ = extended_state.head(X_hat_.size());
        F_hat_ = extended_state.tail(F_hat_.size());
    }

    void update(const Eigen::VectorXd& Y) {
        Eigen::MatrixXd Kk = compute_Kk();
        Eigen::VectorXd extended_state(X_hat_.size() + F_hat_.size());
        extended_state << X_hat_, F_hat_;
        extended_state = extended_state - Kk * (Y - C_ * extended_state);
        X_hat_ = extended_state.head(X_hat_.size());
        F_hat_ = extended_state.tail(F_hat_.size());

        P_ = (1 + theta_) * (A_ + Kk * C_) * P_ * (A_ + Kk * C_).transpose() + Kk * R_ * Kk.transpose()
            + (1 + 1 / theta_) * Q1_ + Q2_;
    }

    int state_size_;
    int extended_size_;
    int measurement_size_;
    int control_size_;
    Eigen::MatrixXd R0_,R1_;
    
private:
    Eigen::VectorXd X_hat_, F_hat_, G_hat_, qk_;  // 状态估计、扩展状态估计、非线性估计、已知参数 qk
    Eigen::MatrixXd P_, A_, B_, Bu_, C_;              // 误差协方差矩阵和系统矩阵
    Eigen::MatrixXd Q1_, Q2_, Qk_, R_;           // 噪声协方差矩阵
    double theta_;                               // 参数 θ
};

class DisturbanceObserver
{
public:
    DisturbanceObserver(double dt);
    DisturbanceObserver() {};   
    ~DisturbanceObserver() {};

    ESKF eskf_{5, 5, 4, 5};

    void eskf_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob, const bool whichR = 0);
    
    Eigen::VectorXd getState() const {
        return z_;
    }

    Eigen::VectorXd getDisturbance() const {
        return d_;
    }

    void setFirstZ(const Eigen::VectorXd& z) {
        z_ = z;
        eskf_.init(z_);
    }

    void setDt(double dt) {
        dt_ = dt;
    }

private:
    double dt_;               
    Eigen::VectorXd z_;       
    Eigen::VectorXd d_;    
};


