#include "ros/ros.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "limo_navigator");

    //Simple Action Client
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    for(int x{}; x < 4; ++x){
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        float waypoint[4][3] = {
            {0, -1.2, 1},
            {-1.0, -1.8, 1},
            {-3.7, -0.5, 1},
            {-2, 0.7, 1}
        };

        goal.target_pose.pose.position.x = waypoint[x][0];
        goal.target_pose.pose.position.y = waypoint[x][1];
        goal.target_pose.pose.orientation.w = waypoint[x][2];

        ac.sendGoal(goal);
    
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved to [%.2f, %.2f]", waypoint[x][0], waypoint[x][1]);
        else
            ROS_INFO("The base failed to move to [%.2f, %.2f] for some reason", waypoint[x][0], waypoint[x][1]);
    }
    return 0;
}