#include <ros/ros.h>
#include <myo_raw/Gesture.h>
#include <std_msgs/Int32.h>

ros::Publisher repub;

void republish_callback(const myo_raw::Gesture&);

/** @function main */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "republish_geom_node");
    ros::NodeHandle n;
    
    repub = n.advertise<std_msgs::Int32>("/myo_raw/gesture_num", 1000);

    ros::Subscriber raw_point = 
        n.subscribe("/myo/gesture", 10, republish_callback);
    
    ros::spin();

    return 0;
}

void republish_callback(const myo_raw::Gesture& msg)
{
    ROS_INFO("trying to post");
    std_msgs::Int32 pose_num;
    pose_num.data = msg.pose_number;
    repub.publish(pose_num);
}
