#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

class VisualServoing
{
private:
    double horizontal_fov_ = 1.5009831567; // °
    double vertical_fov_ = 0.971;   // rad
    double fx_ = 454.68;   //691.4414
    double fy_ = 454.68;   //714.9955
    double  image_height_= 480.0;  // 576
    double  image_width_= 848.0;  // 720
    double cx_ = image_width_ / 2;
    double cy_ = image_height_ / 2;
    

    Eigen::Matrix3d rb_c_;
    Eigen::Vector3d pb_c_{0.108, 0, -0.01};
    Eigen::Matrix<double, 6, 6> adj_tbc_ = Eigen::Matrix<double, 6, 6>::Zero();

    // opencv
    int lowThreshold_ = 100;
    const int ratio_ = 3;
    const int kernel_size_ = 3;
    int roi_height_ = 150;
    cv::Rect roi_;
    cv::Mat result_window_ = cv::Mat::zeros(image_height_, image_width_, CV_8UC3);
    std::string outputfile_ = "/home/lw/output_video.avi";
    int framerate_ = 30;
    cv::VideoWriter videowriter_;
    // double desired_x1_ = 0.0;
    // double desired_x2_ = 0.0;
    // double desired_height_ = 3.0;
    const double cruise_speed_ = 0.5; // m/s
    Eigen::Vector<double, 7> gain_;
    Eigen::Matrix<double, 2, 6> jacobian_;

    double theta_;
    double r_;
    double d_;
    double desired_theta_ = M_PI_2;
    double desired_r_ = 0.0;
    double desired_d_ = 300.0;

    Eigen::Vector<double, 6> vb_;
    Eigen::Matrix<double, 6, 1> v_null_;

    Eigen::Vector<cv::Point2d, 2> mid_line_;
    double d1_, d2_, d3_, d4_, d5_, d6_, dm_;
    const double pipe_diameter_ = 1.5; // m

public:
    VisualServoing();
    ~VisualServoing();

    void visualServoing(const cv::Mat& src);
    void calculateJacobian();
    cv::Mat edgeDetection(const cv::Mat& src);
    std::pair<cv::Point, cv::Point> edgePointDetection(const cv::Mat& src, const int row_num);
    void imageProcess(const cv::Mat& src);
    void pixelPlane2ImagePlane(double u, double v, double &x, double &y);
    void calculateTargetVelocity();
    Eigen::Matrix<double, 6, 1> getNullVelocity();
    Eigen::Vector<double, 6> getVelocity();
    Eigen::Matrix<double, 2, 6> getJacobian();
    std::pair<double, double> getStates();
    void image2Video(const cv::Mat& src);
    double getD();
    double image_width_norm_ = (image_width_-cx_) / fx_;
    cv::Mat getResultWindow()
    {
        return result_window_;
    }

    int getRoiHeight()
    {
        return roi_height_;
    }

    double getImageHeight()
    {
        return image_height_;
    }
    double getImageWidth()
    {
        return image_width_;
    }

    double getFrontWidth()
    {
        return d1_;
    }
};