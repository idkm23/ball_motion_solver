#ifndef TIPFINDER_H 
#define TIPFINDER_H

#include <ros/ros.h>

//image transport/conversion libraries
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <queue>

/**
 * Finds the tip in the camera's context using color detection
 * performs a perspective transform on the found point and publishes it
 */
class TipFinder {

private:
    ros::NodeHandle n;

    /* updates the ball's location based upon any object detection */
    ros::Publisher tip_point_pub, n_missing_pub;
    
    /* fetches the scene to detect the image */
    ros::Subscriber img_sub; 

    /* Thresholding values for color detection */
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;
    int iLowH_hand, iHighH_hand, iLowS_hand, iHighS_hand, iLowV_hand, iHighV_hand;

    /* warp perspective matrix */
    static const cv::Mat H;

    /* Coordinate constant to express when the box is not found */
    static const int NOT_FOUND_COORD;

    /* Use other methods to find target */
    static const bool FULL_DETECTION;

    /* a queue to save previous positions, may be used to predict curret position.*/
    std::queue<geometry_msgs::Point> point_tracer;

    /* number of missing detections */
    std_msgs::Int32 n_missing;

public:
    
    /**
     * TipFinder constructor, instantiates ros objects
     */
    TipFinder(); 

    /**
     * img_sub's callback, finds the location of a tip every 1 in three frames
     * @param ros_img, image of object in a ros msg
     */
    void img_callback(const sensor_msgs::ImageConstPtr&);
    
    /** compute centroid of a mass
       takes a thresholded image, and a threshold value for area
    */
    geometry_msgs::Point get_centroid(cv::Mat&, int areaThreshold);

    /* predict position */
    geometry_msgs::Point get_velocity();
};

#endif
