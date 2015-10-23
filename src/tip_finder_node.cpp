#include "TipFinder.cpp"

/** @function main */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "tip_finder_node");
    TipFinder t;
    
    ros::spin();
    return 0;
}
