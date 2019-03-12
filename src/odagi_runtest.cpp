#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <unistd.h>

#include <tf/transform_datatypes.h>

typedef struct
{
	double x, y, theta;
	double v, w;
} o_odm;

int gShutOff = 0;
	// @brief シグナルハンドラ
	void ctrlC( int aStatus )	
	{
		signal( SIGINT, NULL );
		gShutOff = 1;
	}
	// @brief Ctrl+cをキャッチするための設定
	void setSigInt( )
	{
		struct sigaction sig;
		memset( &sig, 0, sizeof ( sig ) );
		sig.sa_handler = ctrlC;
		sigaction( SIGINT, &sig, NULL );
	}

class RsjRobotTestNode
{	

private:
	nav_msgs::Odometry odom_;
  
	ros::NodeHandle nh_;

	ros::Subscriber sub_odom_;
	ros::Publisher pub_twist_;
  
	//ros::Publisher pub_odom_;
  
	o_odm odm_gl;
	// ここに速度指令の出力コード
	geometry_msgs::Twist cmd_vel;
  
	//geometry_msgs::Point cmd_pos;

	//オドメトリ作成
	void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
	{
		odm_gl.x = msg->pose.pose.position.x;
		odm_gl.y = msg->pose.pose.position.y;
		odm_gl.theta = tf::getYaw(msg->pose.pose.orientation);
		odm_gl.v = msg->twist.twist.linear.x;
		odm_gl.w = msg->twist.twist.angular.z;
	  
		odom_ = *msg;
	}
	//over_line---------------------------------------------------------------------------
	bool odagi_over_line( double Xg, double Yg, double THg )
	{
		ros::spinOnce();
		//距離を判定
		double H = (( Xg - odm_gl.x ) * sin( M_PI_2 - THg )) + (( Yg - odm_gl.y ) * cos( M_PI_2 - THg ));
		
		if( H < 0 ){
			return true;		
		}
		return false;
	}
	//-------------------------------------------------------------------------------------------------
  
	//near angle-------------------------------------------------------------------
	bool odagi_near_angle( double THg, double Tole )
	{
	  
		ros::spinOnce();
		double angle = odm_gl.theta;
		double dist = THg - angle;
		
		if(fabs(dist) < Tole ){
			return true;
		}
		return false;
	}
	//--------------------------------------------------------------------------------------
  
	//line------------------------------------------------------------------------
	void odagi_line( double Xg, double Yg, double THg )
	{
		ros::spinOnce();
		double K1 = -1.0, K2 = 1.0;
		
		double distance = (( odm_gl.x - Xg ) * sin( THg )) + (( odm_gl.y - Yg ) * cos( THg ));
					
		// Calculate desired angle (th_d = ....)
		double th_d = K1 * distance ;
		// Calculate desired angluar velocity ( w_d = ... )
		double w_d = K2 * ( th_d + THg - odm_gl.theta ) ;
		
		if( w_d > M_PI ){
			w_d = w_d - 2 * M_PI;
		}
		else if( w_d < -M_PI ){
			w_d = w_d + 2 * M_PI;
		}
			
		// Send the desired angluar velocity to the motor driver
		cmd_vel.linear.x = 0.2;
		cmd_vel.angular.z = w_d;
		pub_twist_.publish(cmd_vel);
	}
	//-----------------------------------------------------------------
  
	//spin----------------------------------------------------
	void odagi_spin( double THg )
	{
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0.2;
		pub_twist_.publish(cmd_vel);
	}
	//----------------------------------------------------------------
  
	//stop------------------------------------------------
	void odagi_stop( )
	{	
		//for (int i = 0; i < 10; i++){
			ros::spinOnce();
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0.0;
			pub_twist_.publish(cmd_vel);
		//}
	}
	//------------------------------------------------------
	

public:
	RsjRobotTestNode()
		: nh_()
	{
		pub_twist_ = nh_.advertise<geometry_msgs::Twist>(
			"cmd_vel", 5);
		sub_odom_ = nh_.subscribe(
			"odom", 5, &RsjRobotTestNode::cbOdom, this);
	}
	
	
	
	void mainloop()
	{
		setSigInt();

		ROS_INFO("Hello ODAGI World!");
		
		//通信速度の設定
		ros::Rate rate(10.0);
		  
		//前進
		while( !odagi_over_line( 1.0, 0.0, 0.0 )  ){
			odagi_line( 1.0, 0.0, 0 );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
	  
		//回転
		while( !odagi_near_angle( M_PI / 2, M_PI / 36.0 )  ){
			odagi_spin( M_PI / 2 );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//前進
		while( !odagi_over_line( 1.0, 0.5, M_PI / 2 )  ){
			odagi_line( 1.0, 0.5, M_PI / 2 );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//回転
		while( !odagi_near_angle( M_PI , M_PI / 36.0 )  ){
			odagi_spin( M_PI  );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//前進
		while( !odagi_over_line( 0.0, 0.5, M_PI )  ){
			odagi_line( 0.0, 0.5, M_PI );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//回転
		while( !odagi_near_angle( -M_PI / 2 , M_PI / 36.0 )  ){
			odagi_spin( -M_PI / 2  );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//前進
		while( !odagi_over_line( 0.0, 0.0, -M_PI / 2 )  ){
			odagi_line( 0.0, 0.0, -M_PI / 2 );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		  
		//回転
		while( !odagi_near_angle( 0 , M_PI / 36.0 )  ){
			odagi_spin( 0  );
			rate.sleep();
			if (gShutOff){
				goto OUT;
			}
		}
		
		// ここに終了処理のコード
		OUT:
		odagi_stop();
		ROS_INFO("finish");
	}
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odagi_runtest");

	RsjRobotTestNode robot_test;

	robot_test.mainloop();
 
}
