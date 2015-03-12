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

        CoinDetector() {

        }

        ~CoinDetector() {

        }

        void detectAndDraw(Mat& img)
        {
            Mat gray;
            vector<Vec3f> circles;
            int numCircles = 0;

            cvtColor( img, gray, CV_BGR2GRAY );

            // Blur so we don't get false positives
            GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

            /// Apply the Hough Transform to find the circles
            HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 2, 100);

            /// Draw the circles detected
            for( size_t i = 0; i < circles.size(); i++)
            {
                /* If color is within coin range, draw circle
                 * 
                 * Typical copper RGB = (153, 117, 89)
                 * Typical silver RGB = (174, 176, 180)
                 *
                 * Unfortunately these values depend heavily on the lighting.
                 * A silver coin can look practically white with light reflecting
                 * off of it. A copper coin can look very dark with minimal light.
                 */
                Vec3b colorAtPoint = img.at<Vec3b>(Point(circles[i][0], circles[i][1]));

                if (((colorAtPoint.val[0] > 200) && (colorAtPoint.val[2] < 150)) ||
                    ((colorAtPoint.val[1] > 200) && (colorAtPoint.val[2] < 150)) ||
                    ((colorAtPoint.val[2] > 200) && (colorAtPoint.val[1] < 100)) ||
                    ((colorAtPoint.val[0] < 50) && (colorAtPoint.val[1] < 50) && (colorAtPoint.val[2] < 50))) {
                    // If (B > 200) && (R < 150) we have a blue-ish object. Not a coin.
                    // If (G > 200) && (R < 150) we have a green-ish object. Not a coin.
                    // If (R > 200) && (G < 100) we have a red-ish object. Not a coin.
                    // If (B < 50) && (G < 50) && (R < 50) we have a black object. Not a coin.
                    continue;
                }
                else {
                    
                    numCircles++;

                    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                    int radius = cvRound(circles[i][2]);

                    // circle outline
                    circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
                }
            }

            std_msgs::Int64 msg;
            if (numCircles > 0) {
                msg.data = 2;
            }
            else {
                msg.data = 0;
            }
            lightPub.publish(msg);
        }

};



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

    coinDetector.detectAndDraw(cv_ptr->image);

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
