#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Int64.h"

#include <string>
#include <iostream> 
#include <cctype>
#include <iterator>
#include <stdio.h>

using namespace cv;
using namespace std;


ros::Publisher lightPub;

class CoinDetector {

    public:

        //string faceCascadeName = "/home/viki/catkin_ws/src/golddigger_master/src/coin.xml";
        CascadeClassifier faceCascade;

        static const float scale = 2.0;

        CoinDetector() {
            namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
        }

        ~CoinDetector() {

        }

        void detectAndDraw(Mat& img)
        {
            Mat gray;
            vector<Vec3f> circles;

            cvtColor( img, gray, CV_BGR2GRAY );

            // Blur so we don't get false positives
            GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

            /// Apply the Hough Transform to find the circles
            HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 2, 100);

            /// Draw the circles detected
            for( size_t i = 0; i < circles.size(); i++)
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                // circle outline
                circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
            }

            cout << "# circles: " << circles.size() << endl;

            std_msgs::Int64 msg;
            if (circles.size() > 0) {
                msg.data = 1;
            }
            else {
                msg.data = 0;
            }
            lightPub.publish(msg);
        }

};


// ---------- end class definition


image_transport::Subscriber sub;
image_transport::Publisher pub;

CoinDetector coinDetector;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // do face detect stuff
    coinDetector.detectAndDraw(cv_ptr->image);

    // cv_bridge::CvImage out_msg;
    // //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
    // out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    // out_msg.image = cv_ptr->image; // Your cv::Mat
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "detect_coins");
    
    ros::NodeHandle n;
    lightPub = n.advertise<std_msgs::Int64>("light", 1);

    image_transport::ImageTransport it (n);
    sub = it.subscribe("/camera/visible/image", 1, imageCallback);
    pub = it.advertise("/output_video", 1);
    
    if (ros::ok()) {
        ros::spin();
    }
    
    return 0;
}
