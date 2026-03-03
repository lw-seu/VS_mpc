#include "visual_servo.h"
#include "mpc_config.h"
VisualServoing::VisualServoing()
{
    rb_c_  << 0.0,-1.0, 0.0,
            -1.0, 0.0, 0.0,
             0.0, 0.0,-1.0;

    Eigen::Matrix3d tbc_skew;
    tbc_skew << 0,       -pb_c_(2), pb_c_(1),
                pb_c_(2), 0,       -pb_c_(0),
               -pb_c_(1), pb_c_(0), 0;
    adj_tbc_.block<3, 3>(0, 0) = rb_c_;
    adj_tbc_.block<3, 3>(3, 3) = rb_c_;
    // adj_tbc_.block<3, 3>(0, 3) = tbc_skew*rb_c_;

    gain_ << 150.0, 5, 1.0, 50.0, 50.0, 12.0, 400.0;
    gain_ = 1.0 * gain_;

    videowriter_.open(outputfile_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
                      framerate_, cv::Size(image_width_, image_height_));

    if (!videowriter_.isOpened()) 
    {
        std::cerr << "Unable to open video file for writing." << std::endl;
    }
}

VisualServoing::~VisualServoing()
{
    videowriter_.release();
}
 
cv::Mat VisualServoing::edgeDetection(const cv::Mat& src)
{
    cv::Mat dst, detected_edges;
    cv::blur(src, detected_edges, cv::Size(3, 3));
    cv::Canny(detected_edges, detected_edges, lowThreshold_, 
                lowThreshold_*ratio_, kernel_size_);
    return detected_edges;    
}

std::pair<cv::Point, cv::Point> VisualServoing::edgePointDetection(const cv::Mat& src, const int row_num)
{
    cv::Point p1, p2;
    p1.y = row_num;
    p2.y = row_num;
    p1.x = 0;
    p2.x = 0;
    for(int i = 0; i < src.cols; i++)
    {
        if(src.at<uchar>( p1.y, i) != 0)
        {
            p1.x = i;
            break;
        }
    }
    for(int i = src.cols-1; i >= p1.x; i--)
    {
        if(src.at<uchar>(p2.y, i) != 0)
        {
            p2.x = i;
            break;
        }
    }
    return std::make_pair(p1, p2);
}

void VisualServoing::pixelPlane2ImagePlane(double u, double v, double &x, double &y)
{
    x = (u - cx_) / fx_;
    y = (v - cy_) / fy_;

    // x = (u - cx_);
    // y = (v - cy_);
}
void VisualServoing::imageProcess(const cv::Mat& src)
{
    cv::Mat detected_dst, img_roi;
    Eigen::Vector<cv::Point2d, 2> f_line, m_line, b_line;
    Eigen::Matrix<cv::Point2d, 3, 4> fmb; // 4 corner points of front, middle, back area
    std::pair<cv::Point, cv::Point> horizontal_midline;
    double f_theta, m_theta, b_theta; // 3 angle of 3 area
    std::pair<cv::Point, cv::Point> midline_point;
    roi_.x = 0;
    roi_.y = static_cast<int>(src.rows/2)-static_cast<int>(roi_height_/2);
    roi_.width = src.cols;
    roi_.height = roi_height_;

    detected_dst = edgeDetection(src);
    std::tie(fmb(0,0), fmb(0,1)) = edgePointDetection(detected_dst, 0);
    std::tie(fmb(0,2), fmb(0,3)) = edgePointDetection(detected_dst, 0+roi_.height-1);
    std::tie(fmb(1,0), fmb(1,1)) = edgePointDetection(detected_dst, roi_.y);
    std::tie(fmb(1,2), fmb(1,3)) = edgePointDetection(detected_dst, roi_.y+roi_.height-1);
    std::tie(fmb(2,0), fmb(2,1)) = edgePointDetection(detected_dst, src.rows-1-roi_.height+1);
    std::tie(fmb(2,2), fmb(2,3)) = edgePointDetection(detected_dst, src.rows-1);
    f_line << cv::Point2d(static_cast<double>((fmb(0,0).x+fmb(0,1).x)/2), fmb(0,0).y),
              cv::Point2d(static_cast<double>((fmb(0,2).x+fmb(0,3).x)/2), fmb(0,2).y);
    m_line << cv::Point2d(static_cast<double>((fmb(1,0).x+fmb(1,1).x)/2), fmb(1,0).y),
              cv::Point2d(static_cast<double>((fmb(1,2).x+fmb(1,3).x)/2), fmb(1,2).y);
    b_line << cv::Point2d(static_cast<double>((fmb(2,0).x+fmb(2,1).x)/2), fmb(2,0).y),
              cv::Point2d(static_cast<double>((fmb(2,2).x+fmb(2,3).x)/2), fmb(2,2).y);
    f_theta = std::atan2(std::abs(f_line[1].y - f_line[0].y), std::abs(f_line[1].x - f_line[0].x));
    m_theta = std::atan2(std::abs(m_line[1].y - m_line[0].y), std::abs(m_line[1].x - m_line[0].x));
    b_theta = std::atan2(std::abs(b_line[1].y - b_line[0].y), std::abs(b_line[1].x - b_line[0].x));

    d1_ = (fmb(0,1).x - fmb(0,0).x) * std::sin(f_theta);
    d2_ = (fmb(0,3).x - fmb(0,2).x) * std::sin(f_theta);
    d3_ = (fmb(1,1).x - fmb(1,0).x) * std::sin(m_theta);
    d4_ = (fmb(1,3).x - fmb(1,2).x) * std::sin(m_theta);
    d5_ = (fmb(2,1).x - fmb(2,0).x) * std::sin(b_theta);
    d6_ = (fmb(2,3).x - fmb(2,2).x) * std::sin(b_theta);

    horizontal_midline = edgePointDetection(detected_dst, src.rows/2-1);
    dm_ = (horizontal_midline.second.x - horizontal_midline.first.x) * std::sin(m_theta);

    pixelPlane2ImagePlane(m_line(0).x, m_line(0).y, mid_line_(0).x, mid_line_(0).y);
    pixelPlane2ImagePlane(m_line(1).x, m_line(1).y, mid_line_(1).x, mid_line_(1).y);

    // printf("d1: %f, d2: %f, d3: %f, d4: %f, d5: %f, d6: %f\n", d1, d2, d3, d4, d5, d6);
    // printf("x1: %f, y1: %f, x2: %f, y2: %f\n", mid_line_(0).x, mid_line_(0).y, mid_line_(1).x, mid_line_(1).y);

    cv::cvtColor(detected_dst, result_window_, cv::COLOR_GRAY2BGR);
    cv::line(result_window_, m_line(0), m_line(1), cv::Scalar(0,0,255), 2);
    cv::rectangle(result_window_, cv::Point(0, 0), cv::Point(image_width_ - 1, image_height_ - 1),  cv::Scalar(0,255,0), 2);
    cv::line(result_window_, cv::Point(image_width_/2, 0), cv::Point(image_width_/2, image_height_), cv::Scalar(50, 255, 255), 1);
}

void VisualServoing::calculateJacobian()
{
    double x1, y1, x2, y2;
    double z1, z2;
    x1 = mid_line_(0).x;
    y1 = mid_line_(0).y;
    x2 = mid_line_(1).x;
    y2 = mid_line_(1).y;
    z1 = pipe_diameter_ * fx_ / d3_;
    z2 = pipe_diameter_ * fx_ / d4_;
    // z1 = z2 = 1.0;
    Eigen::Matrix<double, 4, 6> J_img; // 物体雅可比 4 点图像 (4*2 x 6) 8x6 测量矩阵
    J_img << -(1/z1), 0.0,     x1/z1, x1*y1,     -(1+x1*x1), y1,
             0.0    , (-1/z1), y1/z1, 1+(y1*y1), -(x1*y1),  -x1,
             -(1/z2), 0.0,     x2/z2, x2*y2,     -(1+x2*x2), y2,
             0.0    , (-1/z2), y2/z2, 1+(y2*y2), -(x2*y2),  -x2;

    double m = (x2-x1)/(y2-y1);
    theta_ = std::atan2((y2-y1),-(x2-x1));
    r_ = sin (theta_) * (x1-(m)*y1);

    Eigen::Matrix<double, 1, 5> J_raux;
    J_raux << -sin(theta_)*(y1/(y1 - y2) - 1), 
              -sin(theta_)*((x1 - x2)/(y1 - y2) - (y1*(x1 - x2))/pow((y1 - y2),2)),
               (y1*sin(theta_))/(y1 - y2),
              -(y1*sin(theta_)*(x1 - x2))/pow((y1 - y2),2),
               abs(cos(theta_))*(x1 - (y1*(x1 - x2))/(y1 - y2));
               
    Eigen::Matrix<double, 2, 5>  J_r;
    J_r << J_raux(0), J_raux(1), J_raux(2), J_raux(3), J_raux(4),
                  0,          0,         0,         0,         1;

    Eigen::Matrix<double, 5, 4> J_theta;
    Eigen::Matrix<double, 1, 4> J_theta_aux;

    J_theta_aux << (y1-y2),-(x1-x2),-(y1-y2),(x1-x2);
    J_theta_aux = 1/(pow((y2-y1),2)+pow((x2-x1),2)) * J_theta_aux;

    J_theta << 1, 0, 0, 0, 
               0, 1, 0, 0, 
               0, 0, 1, 0,
               0, 0, 0, 1,
               J_theta_aux(0), J_theta_aux(1), J_theta_aux(2), J_theta_aux(3);

    jacobian_ = J_r * J_theta * J_img * adj_tbc_.inverse();

    // std::cout << "jacobian_: " << std::endl << jacobian_ << std::endl;
}


void VisualServoing::calculateTargetVelocity()
{
    Eigen::Vector2d r_theta;

    double err_r = desired_r_ - r_;
    // err_r = -err_r;
    double err_theta = desired_theta_ - theta_;
    // printf("err_r_: %f, err_theta_: %f\n", err_r, err_theta);
    
    r_theta << gain_(0)* (err_r), 
               gain_(1)* (err_theta);

    double r_aux = (err_r)/(image_width_/2);
    //float r_aux = tanh((err_r)/gain_(5,0));

    Eigen::Vector2d r_th_norm;
    r_th_norm << gain_(0)* r_aux, 
                 gain_(1)* (err_theta );

    // Eigen::MatrixXf r_th_wo_gain(2,1);
    // r_th_wo_gain << r_aux, 
    //                 err_theta;
    
    Eigen::Matrix<double, 6, 6> gain_null;
    gain_null << 1 ,0 ,0 ,0 ,0 ,0
                ,0 ,1 ,0 ,0 ,0 ,0
                ,0 ,0 ,1 ,0 ,0 ,0
                ,0 ,0 ,0 ,gain_(3) ,0 ,0
                ,0 ,0 ,0 ,0 ,gain_(4) ,0
                ,0 ,0 ,0 ,0 ,0 ,1;
                
    gain_null = gain_(2)*gain_null ;

    double alpha1 = 0.0, alpha2 = 0.0;
    d_ = alpha1 * (d1_+d2_)/2 + alpha2 * (d5_+d6_)/2 + (1-alpha1-alpha2) * (d3_+d4_)/2;
    // printf("d_ = %f\n", d_);
    // v_null_ << cruise_speed_,
    //           0,
    //           gain_(6)*(1/desired_d_- 1/d_) + 
    //           (1-dm_/d1_) * cruise_speed_,
    //           0,
    //           0,
    //           0;    

    v_null_ << cruise_speed_,
              0,
              gain_(6)*(1/desired_d_- 1/d_)+ 
              1.5*(d1_-dm_)/image_height_*2 * cruise_speed_,
              0,
              0,
              0;    
    
    
    Eigen::Matrix<double, 6, 6> Iden = Eigen::Matrix<double, 6, 6>::Identity();
 
    Eigen::Matrix<double, 6, 2> J_sal_trans = jacobian_.transpose();

    Eigen::Matrix<double, 2, 2> in_J_aux = jacobian_ * gain_null.inverse()* J_sal_trans;
    Eigen::Matrix<double, 6, 2> in_J_sal;
    in_J_sal = gain_null.inverse()*J_sal_trans * in_J_aux.inverse();

    vb_ = in_J_sal * r_th_norm + (Iden - in_J_sal* jacobian_ ) * v_null_ ;
    // printf("vb: %f %f %f %f %f %f\n", vb_(0), vb_(1), vb_(2), vb_(3), vb_(4), vb_(5));
}

void VisualServoing::visualServoing(const cv::Mat& src)
{
    imageProcess(src);
    calculateJacobian();
    calculateTargetVelocity();
}

Eigen::Vector<double, 6> VisualServoing::getVelocity()
{
    return vb_;
}


Eigen::Matrix<double, 2, 6> VisualServoing::getJacobian()
{
    return jacobian_;
}

std::pair<double, double> VisualServoing::getStates()
{
    // return std::make_pair(r_/(image_width_/2), theta_);
    return std::make_pair(r_, theta_);
}

void VisualServoing::image2Video(const cv::Mat& src)
{
    videowriter_.write(src);
}

Eigen::Matrix<double, 6, 1> VisualServoing::getNullVelocity()
{
    return v_null_;
}

double VisualServoing::getD()
{
    return d_;
}