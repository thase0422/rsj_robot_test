#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"

#include <tf/transform_datatypes.h>

#define max_vel 0.5
#define max_angvel 0.5

class OdagiJoystickNode
{
private:
	ros::NodeHandle nh_;

	ros::Subscriber sub_odom_;
	ros::Publisher pub_twist_;
	ros::Subscriber sub_joy_;
	
	// ここに速度指令の出力コード
	geometry_msgs::Twist cmd_vel;

	void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
	{
	}
	  
	void joy_callback(const sensor_msgs::Joy& joy_msg){
		cmd_vel.linear.x =max_vel * joy_msg.axes[1];
		cmd_vel.angular.z=max_angvel * joy_msg.axes[0];
		ROS_INFO("vel=%lf, angular=%lf",cmd_vel.linear.x,cmd_vel.angular.z);
	}

public:
	OdagiJoystickNode()
	: nh_()
	{
		pub_twist_ = nh_.advertise<geometry_msgs::Twist>(
			"cmd_vel", 5);
		sub_odom_ = nh_.subscribe(
			"odom", 5, &OdagiJoystickNode::cbOdom, this);
		sub_joy_ = nh_.subscribe(
			"joy", 5, &OdagiJoystickNode::joy_callback, this);
	}
	void mainloop()
	{
		ROS_INFO("Hello joystick World!");

		ros::Rate rate(10.0);
		while (ros::ok()){
		ros::spinOnce();
		
		pub_twist_.publish(cmd_vel);
		
		rate.sleep();
		}
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joystick_publisher");

  OdagiJoystickNode joystick_test;

  joystick_test.mainloop();
}
