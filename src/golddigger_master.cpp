#include <ros/ros.h>

#include "Constants.h"
#include "assignment_5/ChangeMode.h"

#include <termios.h>

// non-blocking keyboard input
// from http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering   
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_mode_keyboard");
    ros::NodeHandle n;
    
    ros::Rate loopRate(10);
    
    ros::ServiceClient changeModeClient = n.serviceClient<assignment_5::ChangeMode>("/change_mode");
    
    while (ros::ok()) {
        
        int c = getch();
        if (c == MODE_1_KEY || c == MODE_2_KEY) {
            assignment_5::ChangeMode srv;
            srv.request.key = c;
            
            if (changeModeClient.call(srv)) {
                if (srv.response.changed) {
                    ROS_INFO("Changed mode to %s", srv.response.mode.c_str());
                }
            }
            else {
                ROS_ERROR("Failed to call service change_mode");
            }
        }
        
        ros::spinOnce();
        loopRate.sleep();
        
    }
    
    return 0;
}