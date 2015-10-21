#ifndef HOMOGRAPHY_H
#define HOMOGRAPHY_H

#include <ros/ros.h>

//image transport/conversion libraries
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/nonfree/features2d.hpp>


/**
 * Homography class detects a provided object in a scene and
 * publishes a finding to the '/sony_box_selection' topic
 */
class Homography {

private:
    ros::NodeHandle n;
    ros::Publisher square_pub;   //updates the bounding box on the glass
    ros::Publisher square_coord_pub; //updates the bounding box based upon any object detection
    ros::Subscriber raw_img_sub; //fetches the scene to detect the image
    ros::Subscriber obj_img_sub; //fetches obj_img and enables raw_img_sub
 
    cv::Mat img_obj; //the image of the object we are trying to find
    
public:
    Homography();
    void raw_img_callback(const sensor_msgs::ImageConstPtr&);
    void obj_img_callback(const sensor_msgs::ImageConstPtr&);
    void pub_square(cv::Point2f);
};

#endif
