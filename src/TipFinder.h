#ifndef TIPFINDER_H 
#define TIPFINDER_H

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
 * Finds the tip in the camera's context using color detection
 * performs a perspective transform on the found point and publishes it
 */
class TipFinder {

private:
    ros::NodeHandle n;
    ros::Publisher tip_point_pub, warped_img_pub; //updates the bounding box based upon any object detection
    ros::Subscriber img_sub; //fetches the scene to detect the image

    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    int skipFrame; //skips frames to try and reduce the glass from crashing

    /* warp perspective matrix */
    static const cv::Mat H;

    /* scale factor for warp matrix */
    static const double s;
 
    /* translational factors */
    static const double XTRANS, YTRANS;
    static const int NOT_FOUND_COORD;

public:
    TipFinder();
    void img_callback(const sensor_msgs::ImageConstPtr&);
};

#endif
