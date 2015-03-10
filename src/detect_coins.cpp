#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_coins");
    ros::NodeHandle n;
    
    ros::Rate loopRate(10);
    
    while (ros::ok()) {


        
        ros::spinOnce();
        loopRate.sleep();
        
    }
    
    return 0;
}