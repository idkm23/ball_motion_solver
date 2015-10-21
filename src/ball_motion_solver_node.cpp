#include "BallMotionSolver.cpp"

/** @function main */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "ball_motion_solver_node");
    BallMotionSolver h;
    
    ros::spin();
    return 0;
}
