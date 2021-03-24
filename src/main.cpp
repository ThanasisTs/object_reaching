#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>

ros::Publisher cmd_vel_pub;
float yGoal, init_x, init_y, init_z, desiredVel;
int D;
geometry_msgs::PointStamped current_pos, init_pos;
geometry_msgs::Twist cmd_vel;
bool wait = true, not_reached_init = true, not_reached_goal = true;
double init_time, final_time;

double euclidean_distance(const geometry_msgs::PointStamped& p1, const geometry_msgs::PointStamped& p2){
	return sqrt(pow(p1.point.x-p2.point.x, 2) + pow(p1.point.y-p2.point.y, 2) + pow(p1.point.z-p2.point.z, 2));
}

void state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr msg){
	current_pos.point = msg->pose.position;
	current_pos.header = msg->header;
	if (not_reached_init){
		if (euclidean_distance(current_pos, init_pos) > 0.001){
			cmd_vel.linear.x = D*(init_pos.point.x - current_pos.point.x);
			cmd_vel.linear.y = D*(init_pos.point.y - current_pos.point.y);
			cmd_vel.linear.z = D*(init_pos.point.z - current_pos.point.z);
			cmd_vel_pub.publish(cmd_vel);
		}
		else{
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel_pub.publish(cmd_vel);
			not_reached_init = false;
			ROS_INFO_STREAM("Reached initial point");
		}
	}
	else{
		if (wait){
			ros::Duration(3).sleep();
			wait = false;
			init_time = ros::Time::now().toSec();
		}
		cmd_vel.linear.x = 0;
		cmd_vel.linear.y = desiredVel + D*(yGoal - current_pos.point.y + 0.032);
		cmd_vel.linear.z = 0;
		cmd_vel_pub.publish(cmd_vel);
		std::cout << msg->twist.linear.y << std::endl;
		if (abs(current_pos.point.y - yGoal - 0.032) < 0.003 and not_reached_goal){
			ROS_INFO_STREAM("Time duration: " << ros::Time::now().toSec() - init_time << "\n");
			not_reached_goal = false;
		}
	}	
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ee_control");
	ros::NodeHandle nh;

	nh.param("ee_control/yGoal", yGoal, 0.0f);
	nh.param("ee_control/desiredVel", desiredVel, 0.0f);
	nh.param("ee_control/D", D, 0);
	nh.param("ee_control/init_x", init_x, 0.0f);
	nh.param("ee_control/init_y", init_y, 0.0f);
	nh.param("ee_control/init_z", init_z, 0.0f);
	init_pos.point.x = init_x;
	init_pos.point.y = init_y;
	init_pos.point.z = init_z;

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ur3_cartesian_velocity_controller/command_cart_vel", 10);
	ros::Subscriber ee_state_sub = nh.subscribe("/ur3_cartesian_velocity_controller/ee_state", 10, state_callback);

	ros::spin();


}