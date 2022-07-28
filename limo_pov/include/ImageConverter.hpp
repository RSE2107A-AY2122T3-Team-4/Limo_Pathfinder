#ifndef __IMAGE_CONVERTER__
#define __IMAGE_CONVERTER__

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <cmath>

class ImageConverter{
public:
    //Default Constructor
    ImageConverter(std::string window_name, std::string img_sub = "", std::string img_pub = "", std::string topic_pub = "") :
        it_(nh_), P(0), I(0), D(0), e(0), le(0)
    {
        ROS_INFO("Establishing Connections");
        winName = window_name;
        //Register video feed if available
        if(!img_sub.empty()) image_sub_ = it_.subscribe(img_sub, 1, &ImageConverter::imageCb, this);
        //Register video out if available
        if(!img_pub.empty()) image_pub_ = it_.advertise(img_pub, 1);
        //Register Tele-op topic if available
        if(!topic_pub.empty()) pub_ = nh_.advertise<geometry_msgs::Twist>(topic_pub, 1000);
        //Register a pop-up window
        cv::namedWindow(winName);
    }
    //Default Destructor
    ~ImageConverter(){
        //Close pop-up window
        cv::destroyWindow(winName);
    }
    //Callback Function
    void imageCb(const sensor_msgs::ImageConstPtr& img){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            //Copy and convert from ROS to OpenCV
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //Update frame
        cv::imshow(winName, cv_ptr->image);
        //Delay until key or timeout in milliseconds
        cv::waitKey(1);

        //Crop Image
        int row[] = {100, 300};
        int col[] = {150, 350};
        Cropped = cv_ptr->image(cv::Range(row[0], row[1]), cv::Range(col[0], col[1]));
        
        //Extract Color
        cv::cvtColor(Cropped, HSV, cv::COLOR_BGR2HSV);
        cv::Scalar white_HSV[2] {
            {  0,   0, 255},
            {255, 255, 255}
        };
        cv::inRange(HSV, white_HSV[0], white_HSV[1], White);
        
        //Apply Gaussian Blur
        cv::GaussianBlur(White, Gaussian, cv::Size(15, 15), 0.0);
        
        //Apply Canny Filter
        int hysteresis[] = {100, 150};
        cv::Canny(Gaussian, Canny, hysteresis[0], hysteresis[1]);
        
        //Apply Hough Probability Transform
        double rho = 1, theta = (CV_PI / 180), minLineLength = 100, maxLineGap = 30;
        int threshold = 80;
        cv::HoughLinesP(Canny, HoughLines, rho, theta, threshold, minLineLength, maxLineGap);
        
        //Calculate Total Length and Gradients
        double length{}, gradient{}, m{}, pos_grad{}, neg_grad{}, grad_num[2]{}, bisect{};
        for(size_t i{}; i < HoughLines.size(); ++i){
            //Pythagorean theorem
            //sqrt(((x2-x1)^2) + ((y2-y1)^2))
            length += sqrt(pow((HoughLines[i][2]-HoughLines[i][0]), 2) + pow((HoughLines[i][3] - HoughLines[i][1]), 2));
            //m = (y2-y1)/(x2-x1)
            m = (HoughLines[i][3] - HoughLines[i][1]) / (HoughLines[i][2] - HoughLines[i][0]);
            if (m >= 0) {
                pos_grad += m; //Positive gradient lines e.g left side lines
                ++grad_num[0];
            }
            if (m <  0) {
                neg_grad += m; //Negative gradient lines e.g right side lines
                ++grad_num[1];
            }
        }
        
        //Average Length
        length = length / HoughLines.size();
        //Average Gradients
        pos_grad = pos_grad / grad_num[0];
        neg_grad = neg_grad / grad_num[1];
        //Bisector Gradient
        bisect = tan( (atan(pos_grad) + atan(neg_grad)) / 2);
        
        //Move the platform
        float linear_spd = 0.2;
        float angular_spd = 0.1 + angular_PID(10, 0, 0, bisect); //Kp, Ki, Kd, current_gradient
        move(linear_spd, angular_spd);
    }
    //Move the platform
    void move(float lx, float az){
        geometry_msgs::Twist msg;
        msg.linear.x = lx;
        msg.angular.z = az;
        pub_.publish(msg);
    }
    //Angular PID controller 
    float angular_PID(float Kp, float Ki, float Kd, float gradient){
        e = gradient - 1;   //Calculate error
        P = e;              //Proportional error
        I += e;             //Integral error (Summation of errors)
        D = e - le;         //Differential error
        le = e;             //Keep error value
        return float(Kp*P + Ki*I + Kd*D);
    }
private:
    //NodeHandler
    ros::NodeHandle nh_;
    //Image Transport Object
    image_transport::ImageTransport it_;
    //Image Subscriber
    image_transport::Subscriber image_sub_;
    //Image Publisher
    image_transport::Publisher image_pub_;
    //Tele-op Publisher
    ros::Publisher pub_;
    ros::Rate pub_rate{10};
    //Window Name
    std::string winName{};
    //Image Data
    cv::Mat Cropped{};
    cv::Mat HSV{};
    cv::Mat White{};
    cv::Mat Gaussian{};
    cv::Mat Canny{};
    std::vector<cv::Vec4i> HoughLines{};
    //PID Values
    float P;
    float I;
    float D;
    float e;
    float le;
};

#endif