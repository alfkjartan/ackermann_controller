#ifndef FIND_ROAD_H
#define FIND_ROAD_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

/**
   Class that subscribes to the image stream from the camera on the ackermann vehicle,
   and implements some basic computer vision methods to find the road ahead. 
   The class is header-file only. You only need to include the header file in your code.
 */
class RoadFinder
{
  cv::Point2f middle_of_road_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat thr, gray, hsv, edge;

  std::vector<cv::Vec4i> lines_; // To hold lines found by Hough transform

 public:
 RoadFinder()
   : it_(nh_)
  {
    // Temporary values 
    middle_of_road_.x = 400;
    middle_of_road_.y = 400;

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ackermann_vehicle/camera1/image_raw", 1,
      &RoadFinder::imageCallback, this);
    image_pub_ = it_.advertise("/road_finder/output_video", 1);
    
    cv::namedWindow(OPENCV_WINDOW);
  }



 RoadFinder(const char* topic_name)
   : it_(nh_)
  {
    // Temporary values 
    middle_of_road_.x = 400;
    middle_of_road_.y = 400;

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(topic_name, 1,
      &RoadFinder::imageCallback, this);
    image_pub_ = it_.advertise("/road_finder/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~RoadFinder()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  int imageWidth(){
    if (thr.empty()) { return 800;} // Temporary value
    else {
      return thr.cols;
    }
  }

  int imageHeight(){
    if (thr.empty()) { return 600; } // Temporary value
    else {
      return thr.rows;
    }
  }

  cv::Point2f& getMidpoint() {
    return middle_of_road_;
  }

  /**
     Returns the number of continuous white pixels along the middle column of the thresholded image
     starting from the bottom of the image. 
  */
  int freeRoadAhead() {
    if (thr.empty()) {
      return 0;
    }
    else {
      // Get mid column of thresholded image
      int horizon = imageHeight() / 2;
      int midcol = imageWidth() / 2;
      cv::Mat mid_line = thr.col(midcol);
      
      int k = imageHeight() - 1;


      while ( mid_line.at<uchar>(k) == 255 and k>horizon) {k--;}
      return imageHeight() - k;
    }
  }

  /**
     Returns the lines found in the image 
   */
  std::vector<cv::Vec4i>& getLines() {
    return lines_; 
  }

  /**
     Callback function analyzing the incoming images
  */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
    // Make sure to ignore the marks in the middle of the road. Since the white contains also
    // the orange of the mid stripes, convert to color space 
    // convert image to grayscale
    cv::cvtColor( cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    cv::inRange( hsv, cv::Scalar(0, 0, 200), cv::Scalar(60, 255, 255), thr );
    //cv::cvtColor( cv_ptr->image, gray, cv::COLOR_BGR2GRAY );
    // convert grayscale to binary image
    //cv::threshold( gray, thr, 180,255,cv::THRESH_BINARY );
 
    // find moments of the image
    cv::Moments m = cv::moments(thr,true);
    middle_of_road_.x = m.m10/m.m00;
    middle_of_road_.y =  m.m01/m.m00;

    cv::Canny(thr, edge, 50, 200, 3 ); // detect edges
    cv::HoughLinesP(edge, lines_, 1, CV_PI/180, 50, 50, 10 ); // detect lines

    // Find free road ahead
    int free_road = freeRoadAhead();
    
    // show the image with a point mark at the centroid, and detected lines
    
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, middle_of_road_, 10, CV_RGB(255,0,0));

    // Draw the lines
    int midcol = imageWidth()/2;
    cv::line( cv_ptr->image, cv::Point(midcol, imageHeight()),
	      cv::Point(midcol, imageHeight()-free_road),
	      cv::Scalar(0, 200, 0), 2, cv::LINE_AA );
    
    for( int i = 0; i < lines_.size(); i++ )
    {
      cv::Vec4i l = lines_[i];
      cv::line( cv_ptr->image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
		cv::Scalar(120,0,200), 2, cv::LINE_AA);
    }

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());
  }

  };

#endif 
