
#include "../include/ImageConverter.hpp"

int main(int argc, char** argv){
    ROS_INFO("Initialising Limo_lane_tracer");
    ros::init(argc, argv, "Limo_line_tracer_node");
    ImageConverter ic{"Limo_lane_tracer", "/camera/rgb/image_raw", "", "/cmd_vel"};
    ros::spin();
    return 0;
}
