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

        string faceCascadeName = "/home/viki/catkin_ws/src/golddigger_master/src/coin.xml";
        CascadeClassifier faceCascade;

        static const float scale = 2.0;

        CoinDetector() {
            if (!faceCascade.load(faceCascadeName)) {
                ROS_ERROR("ERROR: Could not load classifier cascade");
            }
        }

        ~CoinDetector() {

        }

        void detectAndDraw(Mat& img)
        {
            int i = 0;
            double t = 0;
            vector<Rect> faces;
            static const Scalar colors[] =  { CV_RGB(0,0,255),
                CV_RGB(0,128,255),
                CV_RGB(0,255,255),
                CV_RGB(0,255,0),
                CV_RGB(255,128,0),
                CV_RGB(255,255,0),
                CV_RGB(255,0,0),
                CV_RGB(255,0,255)} ;
            Mat gray;

            cvtColor( img, gray, CV_BGR2GRAY );
            equalizeHist( gray, gray );

            t = (double)cvGetTickCount();
            faceCascade.detectMultiScale( gray, faces,
                1.1, 2, 0
                //|CV_HAAR_FIND_BIGGEST_OBJECT
                //|CV_HAAR_DO_ROUGH_SEARCH
                |CV_HAAR_SCALE_IMAGE
                ,
                Size(30, 30) );
            t = (double)cvGetTickCount() - t;
            // printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
            
            for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
            {
                Mat smallImgROI;
                Point center;
                Scalar color = colors[i%8];
                int radius;

                double aspect_ratio = (double)r->width/r->height;
                if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
                {
                    center.x = cvRound((r->x + r->width*0.5)*scale);
                    center.y = cvRound((r->y + r->height*0.5)*scale);
                    radius = cvRound((r->width + r->height)*0.25*scale);
                    circle( img, center, radius, color, 3, 8, 0 );
                }
                else
                {
                    rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
                               cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
                               color, 3, 8, 0);
                }
            }

            std_msgs::Int64 msg;
            if (faces.begin() != faces.end()) {
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
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
