#include "find_road.h"
#include "move_car.h"

int main(int argc, char** argv)
{
  float speed = 0.7;  
  float Ka = 1/200;
  if (argc < 3) {
    ROS_ERROR("Wrong number of arguments. Usage:\n rosrun ackermann_controller stay_on_road_node speed Ka");
    return 1;
  } else {
    speed = atof(argv[1]);
    Ka = atof(argv[2]);
  } 

  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  AckermannMover am;
  ros::Rate rate(20);

  while(ros::ok()) {
    // The speed is constant, but the steering angle is proportional
    //to the distance of the middle of the road from the middle of the image

    ros::spinOnce();
    float error = rf.width()/2 - middle_of_road.x; // Error in number of pixels
    am.go(speed, error*Ka);
    rate.sleep();
  }
  return 0;
}
