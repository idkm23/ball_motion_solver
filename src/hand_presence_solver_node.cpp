#include "HandPresenceSolver.cpp"

/** @function main */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "hand_presence_solver_node");
    HandPresenceSolver h;
    
    ros::spin();
    return 0;
}
