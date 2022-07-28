#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Static name for pop-up
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "LIMO's WACKY POV";

//Ros image to OpenCV image class
class ImageConverter{
    //NodeHandler
    ros::NodeHandle nh_;
    //Image Transport Object
    image_transport::ImageTransport it_;
    //Image Subscriber
    image_transport::Subscriber image_sub_;
    //Image Publisher
    image_transport::Publisher image_pub_;
public:
//Default constructor
    ImageConverter() : it_(nh_){
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
        //Display in a pop-up
        cv::namedWindow(OPENCV_WINDOW);
    }
//Default Destructor
    ~ImageConverter(){
        //Close pop-up
        cv::destroyWindow(OPENCV_WINDOW);
    }
//Image callback function
    void imageCb(const sensor_msgs::ImageConstPtr& img){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            //Convert from ROS to OpenCV with a copy
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //Update frame
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //Delay until key or timeout
        cv::waitKey(3);

        cv::Mat BGR[3];
        cv::Mat GRB;
        cv::split(cv_ptr->image, BGR);
        cv::Mat tmp(BGR[0]);//Save Blue Channel
        BGR[0] = BGR[1];//Blue to Green
        BGR[1] = BGR[2];//Green to Red
        BGR[2] = tmp;//Red to Blue
        cv::merge(BGR, 3, GRB);
        cv::namedWindow(OPENCV_WINDOW2);
        cv::imshow(OPENCV_WINDOW2, GRB);

    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "limo_pov_node");
    ImageConverter ic;
    ros::spin();
    return 0;
}