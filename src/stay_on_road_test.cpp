#include "find_road.h"
#include "move_car.h"

int main(int argc, char** argv)
{
  float speed = 0.7;  
  float Ka = 1.0/200;
  if (argc > 1) { speed = atof(argv[1]); }
  if (argc > 2) { Ka = atof(argv[2]); }


  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  AckermannMover am;
  ros::Rate rate(20);

  while(ros::ok()) {
    // The speed is constant, but the steering angle is proportional
    //to the distance of the middle of the road from the middle of the image

    ros::spinOnce();
    float error = rf.width()/2 - rf.get_midpoint().x; // Error in number of pixels
    ROS_INFO("Sending command speed=%f, steering_angle=%f", speed, error*Ka);
    am.go(speed, error*Ka);
    rate.sleep();
  }
  return 0;
}
