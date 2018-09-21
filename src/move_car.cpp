#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Header.h>


class AckermannMover
{
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;  
public:
  AckermannMover() {
    cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_vehicle/ackermann_cmd", 10);
  }

  ~AckermannMover()  { }

  void go(float speed, float steering_angle) {
    //std_msgs::Header header;
    //ackermann_msgs::AckermannDrive drive;
    ackermann_msgs::AckermannDriveStamped driveStamped;
    driveStamped.drive.steering_angle = steering_angle;
    driveStamped.drive.speed = speed;

    cmd_pub_.publish(driveStamped);
  }
};

int main(int argc, char** argv)
{
  float speed = 1.0;  
  float st_a = .2;  

  if (argc < 3) {
    ROS_ERROR("Wrong number of arguments. Usage:\n rosrun ackermann_controller move_car_node speed steering_angle");
    return 1;
  } else {
    speed = atof(argv[1]);
    st_a = atof(argv[2]);
  } 

  ros::init(argc, argv, "move_car");
  AckermannMover am;

  ros::Rate rate(10);

  while(ros::ok()) {
    am.go(speed, st_a);
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
