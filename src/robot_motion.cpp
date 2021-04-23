#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <vector>

ros::Publisher cmd_vel_pub;
float desiredVel;
int D;
std::vector<float> xGoal, yGoal;
geometry_msgs::PointStamped current_pos;
geometry_msgs::Twist cmd_vel;
bool prediction = false, direction;

void prediction_callback(const std_msgs::Bool::ConstPtr msg){
	prediction = true;
	direction = msg->data;
}

void state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr msg){
	current_pos.point = msg->pose.position;
	current_pos.header = msg->header;
	if (prediction){
		if (direction){
			cmd_vel.linear.x = desiredVel + D*(xGoal[0] - current_pos.point.x);
			cmd_vel.linear.y = desiredVel + D*(yGoal[0] - current_pos.point.y);
		}
		else{
			cmd_vel.linear.x = desiredVel + D*(xGoal[1] - current_pos.point.x);
			cmd_vel.linear.y = desiredVel + D*(yGoal[1] - current_pos.point.y);
		}
		cmd_vel.linear.z = 0;
		cmd_vel_pub.publish(cmd_vel);
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_motion");
	ros::NodeHandle nh;

	nh.param("robot_motion/xGoal", xGoal, std::vector<float>(0.0f));
	nh.param("robot_motion/yGoal", yGoal, std::vector<float>(0.0f));
	nh.param("robot_motion/desiredVel", desiredVel, 0.0f);
	nh.param("robot_motion/D", D, 0);

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ur3_cartesian_velocity_controller/command_cart_vel", 10);
	ros::Subscriber ee_state_sub = nh.subscribe("/ur3_cartesian_velocity_controller/ee_state", 10, state_callback);
	ros::Subscriber prediction_sub = nh.subscribe("/prediction", 10, prediction_callback);
	ros::spin();
}