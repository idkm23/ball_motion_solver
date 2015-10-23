#include "TipFinder.h"

using namespace cv;

const Mat TipFinder::H = 
    (Mat_<double>(3,3) << 
        0.00254226, 0.00012822, 0.38870149, 
        -0.00048569, 0.00300231, 0.92131758, 
        -0.00000103, 0.00000039, 0.00832693);

/**
 * TipFinder constructor, instantiates ros objects
 */
TipFinder::TipFinder() {

    img_sub = n.subscribe<sensor_msgs::Image>
        (/*"/ball_mover/camera/image"*/"/usb_cam/image_raw", 10, &TipFinder::img_callback, this);
    tip_point_pub = n.advertise<geometry_msgs::Point>("/ball_mover/tip_point", 1000);

    /* webcam-friendly hsv values 
    iLowH = 39;
    iHighH = 81;
    iLowS = 65;
    iHighS = 222;
    iLowV = 28;
    iHighV = 255;
    */ 

    /* glass-friendly hsv values */ 
    iLowH = 39;
    iHighH = 81;
    iLowS = 65;
    iHighS = 222;
    iLowV = 28;
    iHighV = 255;
    

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window
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

    if(skipFrame++ > 3) {
        skipFrame = 0;
        return;
    }    

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

    Mat cropped_scene = cv_ptr->image;
        
    Mat imgHSV;

    cvtColor(cropped_scene, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

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
    tip_point.x = -1;
    tip_point.y = -1;
    tip_point.z = -1;

    if (dArea > 62000) {
        //finds the center of the hand
        int posX = dM10 / dArea;
        int posY = dM01 / dArea;

        cvtColor(imgThresholded, imgThresholded, COLOR_GRAY2BGR);

        ellipse( imgThresholded, Point(posX, posY), Size(10, 10), 360, 0, 360, Scalar(100, 100, 255), 5, 8);

        //transform point with H matrix
        
        tip_point.x = posX;
        tip_point.y = posY;
    } 
    
    tip_point_pub.publish(tip_point);
    
    imshow("camera", imgThresholded);
    imshow("camera_raw", cropped_scene);
    waitKey(1);
}

