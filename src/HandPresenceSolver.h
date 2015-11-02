#ifndef HANDPRESENCESOLVER_H 
#define HANDPRESENCESOLVER_H

#include <ros/ros.h>

//image transport/conversion libraries
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

/**
 * Determines if the hand is present in a picture
 * using simple color detection
 */
class HandPresenceSolver {

private:
    ros::NodeHandle n;
    ros::Publisher hand_presence_pub; //updates the bounding box based upon any object detection
    ros::Subscriber img_sub; //fetches the scene to detect the image

    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    int skipFrame; //skips frames to try and reduce the glass from crashing
 
public:
    HandPresenceSolver();
    void img_callback(const sensor_msgs::ImageConstPtr&);
};

#endif
