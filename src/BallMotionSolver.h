#ifndef BALLMOTIONSOLVER_H 
#define BALLMOTIONSOLVER_H

#include <ros/ros.h>

//image transport/conversion libraries
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

/**
 * Homography class detects a provided object in a scene and
 * publishes a finding to the '/sony_box_selection' topic
 */
class BallMotionSolver {

private:
    ros::NodeHandle n;
    ros::Publisher ball_motion_pub; //updates the bounding box based upon any object detection
    ros::Subscriber img_sub; //fetches the scene to detect the image

    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    const static double PI;
 
public:
    BallMotionSolver();
    void img_callback(const sensor_msgs::ImageConstPtr&);
    void pub_motion(double, double);
    void calculate_direction_of_motion(int, int, int, int);
};

#endif
