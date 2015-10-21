#include "Homography.h"

using namespace cv;

/**
 * Homography constructor, instantiates ros objects
 */
Homography::Homography() {

    obj_img_sub = n.subscribe<sensor_msgs::Image>
        ("/imgObj/image", 10, &Homography::obj_img_callback, this);
    square_coord_pub = n.advertise<geometry_msgs::Point>("sony_box_selection_topic", 1000);

}

void Homography::pub_square(Point2f top_left_corner) {
    geometry_msgs::Point point;
    point.x = top_left_corner.x;
    point.y = top_left_corner.y;

    square_coord_pub.publish(point);
}
 
/**
 * obj_img_sub's callback, stores image of object in member variable
 * @param ros_img, image of object in a ros msg
 */
void Homography::obj_img_callback(const sensor_msgs::ImageConstPtr& ros_img)
{
    ROS_INFO("Entered obj_img_callback");
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

    img_obj = cv_ptr->image;

    if(raw_img_sub.getTopic().empty()) {
        raw_img_sub = n.subscribe<sensor_msgs::Image>
            (/*"/usb_cam/image_raw"*/ "/camera/image", 10, &Homography::raw_img_callback, this);
    }
}

/**
 * Callback for the scene that we are trying to detect an object in
 * @param ros_img, image scene in sensor_msgs form (ros)
 */
void Homography::raw_img_callback(const sensor_msgs::ImageConstPtr& ros_img) 
{
    if( !img_obj.data )
    {
        ROS_INFO("Skipped frame");
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

    Mat img_scene = cv_ptr->image;

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect( img_obj, keypoints_object );
    detector.detect( img_scene, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_obj, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 20;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //printf("-- Max dist : %f \n", max_dist );
    //printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    { 
        if( matches[i].distance < 3*min_dist )
            { good_matches.push_back( matches[i]); }
    }

    Mat img_matches;
    drawMatches( img_obj, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    Mat H;
    try {
        H = findHomography( obj, scene, RANSAC );
    } catch(cv::Exception& e) {
        return;
    }
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_obj.cols, 0 );
    obj_corners[2] = cvPoint( img_obj.cols, img_obj.rows ); obj_corners[3] = cvPoint( 0, img_obj.rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    Point2f box_center;
    box_center.x = scene_corners[0].x + (scene_corners[1].x - scene_corners[0].x)/2;
    box_center.y = scene_corners[0].y + (scene_corners[3].y - scene_corners[0].y)/2;    

    pub_square(box_center);

    Point2f ptColsX = Point2f(img_obj.cols, 0);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + ptColsX, scene_corners[1] + ptColsX, Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + ptColsX, scene_corners[2] + ptColsX, Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + ptColsX, scene_corners[3] + ptColsX, Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + ptColsX, scene_corners[0] + ptColsX, Scalar( 0, 255, 0), 4 );

    line( img_matches, box_center, box_center + Point2f(0, 1), Scalar(255, 255, 0), 4 );
    
    //-- Show detected matches
    imshow( "Good Matches & Object detection", img_matches );

    waitKey(1);
}
