#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class RoadFinder
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat thr, gray;
  
public:
  RoadFinder()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ackermann_vehicle/camera1/image_raw", 1,
      &RoadFinder::imageCb, this);
    image_pub_ = it_.advertise("/road_finder/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RoadFinder()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Find white part of image. Determine centroid and draw circle at centroid
 
    // convert image to grayscale
    cv::cvtColor( cv_ptr->image, gray, cv::COLOR_BGR2GRAY );
 
    // convert grayscale to binary image
    cv::threshold( gray, thr, 200,255,cv::THRESH_BINARY );
 
    // find moments of the image
    cv::Moments m = cv::moments(thr,true);
    cv::Point p(m.m10/m.m00, m.m01/m.m00);
 
    // coordinates of centroid
    //cout<< Mat(p)<< endl;
 
    // show the image with a point mark at the centroid
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, p, 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  ros::spin();
  return 0;
}
