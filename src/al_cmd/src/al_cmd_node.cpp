#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../../al_odom/src/serial.h"
#include "../../robot_params/params.h"

int controlPort;

ros::Time current_time, last_time;


int16_t constain(int16_t x, int16_t min, int16_t max)
{
	if (x < min)
		return min;

	if (x > max)
		return max;

	return x;
}

#define CMD_RATE 50

int16_t toBE(int16_t val)
{
	return (val << 8) | ((val >> 8) & 0xFF);
}

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
//  ROS_INFO("fwd: %f rot: %f", msg->linear.x, msg->angular.z);
  if ((last_time - current_time).toSec() < (1 / CMD_RATE))
  {
  	ROS_INFO("rate to high, dropping");
  	return;
  }

  last_time = current_time;

  double rotationDist = msg->angular.z / 2 * wheelDistance;

  int16_t speedLeft = metersToEncoder(msg->linear.x - rotationDist) / CMD_RATE;
  int16_t speedRight = metersToEncoder(msg->linear.x + rotationDist) / CMD_RATE;

//  printf("%d\t%d\n", speedLeft, speedRight);

  char buffer[] = { 0, 0, 0, 0, 0, 0 };

  *((int16_t*)(buffer+2)) = toBE(constain(speedLeft, -255, 255));
  *((int16_t*)(buffer+4)) = toBE(constain(speedRight, -255, 255));

  write(controlPort, buffer, sizeof(buffer));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  std::string serialPath;
  ros::param::get("~serial", serialPath);

  controlPort = openSerial(serialPath.c_str());

  last_time = ros::Time::now();
  ros::Subscriber sub = n.subscribe("cmd_vel", 1, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}