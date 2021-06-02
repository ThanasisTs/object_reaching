#include "object_reaching/robot_motion.h"

double euclidean_distance(const geometry_msgs::PointStamped& p1, const geometry_msgs::PointStamped& p2){
	return sqrt(pow(p1.point.x-p2.point.x, 2) + pow(p1.point.y-p2.point.y, 2) + pow(p1.point.z-p2.point.z, 2));
}

void prediction_callback(const std_msgs::Bool::ConstPtr msg){
	prediction = true;
	direction = msg->data;
}

void state_callback(const cartesian_state_msgs::PoseTwist::ConstPtr msg){
	current_pos.point = msg->pose.position;
	current_pos.header = msg->header;

	// Reach the initial robot position
	if (euclidean_distance(current_pos, init_pos) > 0.001 and not reached_init){
		cmd_vel.linear.x = 1*(init_pos.point.x - current_pos.point.x);
		cmd_vel.linear.y = 1*(init_pos.point.y - current_pos.point.y);
		cmd_vel.linear.z = 1*(init_pos.point.z - current_pos.point.z);
		cmd_vel_pub.publish(cmd_vel);
	}
	else{
		reached_init = true;
	}
	
	// Check if the initial robot position has been reached
	if (reached_init){
		// Check if the output of the prediction is available
		if (prediction){
			// Compute the direction of the robot motion according to the prediction
			if (first_time){
				first_time = false;
				if (direction){
					goal_pos.point.x = xGoal[0];
					goal_pos.point.y = yGoal[0];
					dirX = abs(xGoal[0] - current_pos.point.x)/(xGoal[0] - current_pos.point.x);
					dirY = abs(yGoal[0] - current_pos.point.y)/(yGoal[0] - current_pos.point.y);
					theta = atan((yGoal[0] - current_pos.point.y)/(xGoal[0] - current_pos.point.x));
				}
				else{
					goal_pos.point.x = xGoal[1];
					goal_pos.point.y = yGoal[1];
					dirX = abs(xGoal[1] - current_pos.point.x)/(xGoal[1] - current_pos.point.x);
					dirY = abs(yGoal[1] - current_pos.point.y)/(yGoal[1] - current_pos.point.y);
					theta = atan((yGoal[1] - current_pos.point.y)/(xGoal[1] - current_pos.point.x));
				}
			}
			// Compute the robot commanded velocities according to the robot direction 
			if (direction){
				cmd_vel.linear.x = desiredVel*dirX + D*(xGoal[0] - current_pos.point.x);
				cmd_vel.linear.y = desiredVel*dirY + D*(yGoal[0] - current_pos.point.y);
			}
			else{
				cmd_vel.linear.x = desiredVel*dirX + D*(xGoal[1] - current_pos.point.x);
				cmd_vel.linear.y = desiredVel*dirY + D*(yGoal[1] - current_pos.point.y);
			}
			cmd_vel.linear.x = 0;
			cmd_vel.linear.z = 0;
			
			// Check if the robot has reached the goal position
			if (current_pos.point.y - goal_pos.point.y > 0 and goal_pos.point.y == yGoal[1] and not reached_goal){
				ROS_INFO_STREAM("Robot time: " << ros::Time::now().toNSec());
				std_msgs::Time robot_time;
				robot_time.data = ros::Time::now();
				time_pub.publish(robot_time);
				reached_goal = true;
			}
			if (current_pos.point.y - goal_pos.point.y < 0 and goal_pos.point.y == yGoal[0] and not reached_goal){
				ROS_INFO_STREAM("Robot time: " << ros::Time::now().toNSec());
				std_msgs::Time robot_time;
				robot_time.data = ros::Time::now();
				time_pub.publish(robot_time);
				reached_goal = true;
				// std_msgs::Bool reset_game;
				// reset_game.data = true;
				// reset_game_pub.publish(reset_game);
			}
			if (reached_goal and abs(cmd_vel.linear.y) < 0.001){
				prediction = false;
			}
			// Publish the commanded velocities
			cmd_vel_pub.publish(cmd_vel);
		}
		else{
			cmd_vel.linear.x = 0;
			cmd_vel.linear.y = 0;
			cmd_vel.linear.z = 0;
			cmd_vel_pub.publish(cmd_vel);
			if (reached_goal){
				std_msgs::Bool reset_game;
				reset_game.data = true;
				reset_game_pub.publish(reset_game);
				ROS_WARN_STREAM("Reached goal. Gonna sleep for 5 seconds and return to the initial robot position");
				ros::Duration(5).sleep();
				ROS_WARN_STREAM("Returning to the initial position");
				reached_init = false, reached_goal = false, first_time = true;
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_motion");
	ros::NodeHandle nh;

	nh.param("robot_motion/xGoal", xGoal, std::vector<float>(0.0f));
	nh.param("robot_motion/yGoal", yGoal, std::vector<float>(0.0f));
	nh.param("robot_motion/desiredVel", desiredVel, 0.0f);
	nh.param("robot_motion/D", D, 0);
	nh.param("robot_motion/xInit", xInit, 0.0f);
	nh.param("robot_motion/yInit", yInit, 0.0f);
	nh.param("robot_motion/zInit", zInit, 0.0f);

	init_pos.point.x = xInit;
	init_pos.point.y = yInit;
	init_pos.point.z = zInit;

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ur3_cartesian_velocity_controller/command_cart_vel", 10);
	time_pub = nh.advertise<std_msgs::Time>("/robot_time_topic", 10);
	reset_game_pub = nh.advertise<std_msgs::Bool>("/reset_game_topic", 10);
	
	ros::Subscriber ee_state_sub = nh.subscribe("/ur3_cartesian_velocity_controller/ee_state", 10, state_callback);
	ros::Subscriber prediction_sub = nh.subscribe("/prediction", 10, prediction_callback);
	ros::spin();
}