#include "find_road.h"
#include "move_car.h"

int main(int argc, char** argv)
{
  float speed = 0.7;  
  float Ka = 1.0/200;
  float max_speed = 2;
  float min_speed = 0.1;
  
  if (argc > 1) { speed = atof(argv[1]); }
  if (argc > 2) { Ka = atof(argv[2]); }
  if (argc > 3) { max_speed = atof(argv[3]); }
  if (argc > 4) { min_speed = atof(argv[4]); }


  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  AckermannMover am;
  ros::Rate rate(20);

  while(ros::ok()) {
    // The speed is constant, but the steering angle is proportional
    //to the distance of the middle of the road from the middle of the image
 
    ros::spinOnce();
    float error = rf.imageWidth()/2 - rf.getMidpoint().x; // Error in number of pixels
    float speed_command = speed; // Here you could do something smarter
    ROS_INFO("Sending command speed=%f, steering_angle=%f", speed_command, error*Ka);
    am.go(speed_command, error*Ka);
    rate.sleep();
  }
  return 0;
}
