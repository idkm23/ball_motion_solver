#include "TipFinder.h"

using namespace cv;

const int TipFinder::NOT_FOUND_COORD = -1337;
const bool TipFinder::FULL_DETECTION = true;

//warp perspective matrix
const Mat TipFinder::H = 
    (Mat_<double>(3,3) << 
        0.00651836, 0.00032875, 0.38868366, 
        -0.00124530, 0.00769791, 0.92131758, 
        -0.00000263, 0.00000101, 0.00832658);

/**
 * TipFinder constructor, instantiates ros objects
 */
TipFinder::TipFinder() {

    img_sub = n.subscribe<sensor_msgs::Image>
        ("/ball_mover/camera/image"/*"/usb_cam/image_raw"*/, 10, &TipFinder::img_callback, this);
    tip_point_pub = n.advertise<geometry_msgs::Point>("/ball_mover/tip_point", 1000);
    n_missing_pub = n.advertise<std_msgs::Int32>("/ball_mover/n_missing", 1000);

    n_missing.data = 0;
    
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
    
    /* glass-friendly hsv values for hand*/ 
    iLowH_hand = 0;
    iHighH_hand = 43;
    iLowS_hand = 46;
    iHighS_hand = 106;
    iLowV_hand = 20;
    iHighV_hand = 255;

    namedWindow("ControlTip", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    //Create trackbars in "Control" window to dynamically adjust HSV threshold values
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

}

geometry_msgs::Point TipFinder::get_centroid(Mat& imgThresholded, int areaThreshold) {

    geometry_msgs::Point point;
    point.x = NOT_FOUND_COORD;
    point.y = NOT_FOUND_COORD;
    point.z = NOT_FOUND_COORD;

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

    //if the colored mass is large enough, then...
    if (dArea > areaThreshold) {
        //finds the center of the tip within the raw perspective
        int raw_x = dM10 / dArea;
        int raw_y = dM01 / dArea;
       
        Mat raw_point = (Mat_<double>(3, 1) << raw_x, raw_y, 1);
        
        //transform point with H matrix
        Mat warped_point = H*raw_point;        
        int warped_x = warped_point.at<double>(0, 0) / warped_point.at<double>(2, 0);
        int warped_y = warped_point.at<double>(1, 0) / warped_point.at<double>(2, 0);        

        //draws the center of the colored mass in binary image 
        cvtColor(imgThresholded, imgThresholded, COLOR_GRAY2BGR);
        ellipse(imgThresholded, Point(raw_x, raw_y), Size(10, 10), 360, 0, 360, Scalar(100, 100, 255), 5, 8);
        
        point.x = warped_x;
        point.y = warped_y;
        point.z = 1;
    return point;
    }
}

geometry_msgs::Point TipFinder::get_velocity() {
    geometry_msgs::Point velocity, predicted;
    predicted.x = NOT_FOUND_COORD;
    predicted.y = NOT_FOUND_COORD;

    if (point_tracer.size()<=1) return predicted;
    // otherwise, compute the average velocity, pixels/frame
    velocity.x = std::abs ( (point_tracer.back().x - point_tracer.front().x) )
                            / (float)(point_tracer.size() - 1);
    velocity.y = std::abs ( (point_tracer.back().y - point_tracer.front().y) )
                            / (float)(point_tracer.size() - 1);
    
   
    return velocity;
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
    
    geometry_msgs::Point tip_point = get_centroid(imgThresholded, 6000);

    bool reliable_detection = false;
    if (tip_point.x>-1 && tip_point.y>-1) 
        reliable_detection = true;

/*
    if (tip_point.x>-1 && tip_point.y>-1){
        reliable_detection = true;

        if (point_tracer.size() >=5) && n_missing.data<5){
            geometry_msgs::Point previous_velocity = get_velocity();
            if ( std::abs(tip_point.x-point_tracer.back().x) > 300*previous_velocity.x
                || std::abs(tip_point.y-point_tracer.back().y) > 300*previous_velocity.y) {
                reliable_detection = false;
                tip_point.x = NOT_FOUND_COORD;
                tip_point.y = NOT_FOUND_COORD;
            }
        }
    }
*/
    if (reliable_detection)
        n_missing.data = 0; // reset missing counter
    else
        n_missing.data += 1;

    if (FULL_DETECTION && ! reliable_detection ) {
        /* perform hand dectection */
        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH_hand, iLowS_hand, iLowV_hand), Scalar(iHighH_hand, iHighS_hand, iHighV_hand), imgThresholded);
        //Threshold the image

        geometry_msgs::Point hand_point = get_centroid(imgThresholded, 62000);
        
        // Set hand point as tip point for now
        tip_point = hand_point;
    }
    
/*
    if (tip_point.x>-1 && tip_point.y>-1) 
        point_tracer.push(tip_point);
    
    if (point_tracer.size()>5)
        point_tracer.pop();
*/

    tip_point_pub.publish(tip_point);

    // for debug
    n_missing_pub.publish(n_missing);
    
    imshow("camera", imgThresholded);
    imshow("camera_raw", raw_scene);
    waitKey(1);
}

