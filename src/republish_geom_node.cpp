#include <ros/ros.h>
#include <myo_raw/Gesture.h>
#include <std_msgs/Int32.h>

// This file is only necessary because it is difficult to make custom messages in rosjava
// Due to my incompetence, I quickly wrote this republisher to avoid the issue

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

// The gesture message has some other fancy attributes, but the only thing needed
// is the gesture number. Rather than make the Gesture message in rosjava, I just sent it
// through an integer-typed topic
void republish_callback(const myo_raw::Gesture& msg)
{
    ROS_INFO("trying to post");
    std_msgs::Int32 pose_num;
    pose_num.data = msg.pose_number;
    repub.publish(pose_num);
}
