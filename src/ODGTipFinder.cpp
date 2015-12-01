#include "ODGTipFinder.h"

using namespace cv;

const int TipFinder::NOT_FOUND_COORD = -1337;

/**
 * TipFinder constructor, instantiates ros objects
 */
TipFinder::TipFinder() {

    img_sub = n.subscribe<sensor_msgs::Image>
        ("/ball_mover/camera/image"/*"/usb_cam/image_raw"*/, 10, &TipFinder::img_callback, this);
    tip_point_pub = n.advertise<geometry_msgs::Point>("/ball_mover/tip_point", 1000);

    /* webcam-friendly hsv values 
    iLowH = 39;
    iHighH = 81;
    iLowS = 65;
    iHighS = 222;
    iLowV = 28;
    iHighV = 255;
    */ 

    /* glass-friendly hsv values lime-green */ 
    iLowH = 28;
    iHighH = 53;
    iLowS = 83;
    iHighS = 255;
    iLowV = 78;
    iHighV = 255;
    

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window to dynamically adjust HSV threshold values
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

}

/**
 * img_sub's callback, finds the location of a tip every 1 in three frames
 * @param ros_img, image of object in a ros msg
 */
void TipFinder::img_callback(const sensor_msgs::ImageConstPtr& ros_img)
{
    ROS_INFO("Entered img_callback");

    //try-catch converts sensor_msgs::Image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try 
    {   
        cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    }   
    catch (cv_bridge::Exception& e)
    {   
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } 

    Mat raw_scene = cv_ptr->image, imgHSV;

    cvtColor(raw_scene, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
    //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;
    
    geometry_msgs::Point tip_point;
    tip_point.x = NOT_FOUND_COORD;
    tip_point.y = NOT_FOUND_COORD;
    tip_point.z = NOT_FOUND_COORD;

    //if the colored mass is large enough, then...
    if (dArea > 6000) {
        //finds the center of the tip within the raw perspective
        int raw_x = dM10 / dArea;
        int raw_y = dM01 / dArea;
       
        //draws the center of the colored mass in binary image 
        cvtColor(imgThresholded, imgThresholded, COLOR_GRAY2BGR);
        ellipse(imgThresholded, Point(raw_x, raw_y), Size(10, 10), 360, 0, 360, Scalar(100, 100, 255), 5, 8);
        
        tip_point.x = raw_x;
        tip_point.y = raw_y;
    } 
    
    tip_point_pub.publish(tip_point);
    
    imshow("camera", imgThresholded);
    imshow("camera_raw", raw_scene);
    waitKey(1);
}

