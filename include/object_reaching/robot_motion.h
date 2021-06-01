#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>

#include <math.h>
#include <vector>

// Flags used in robot motion
bool prediction = false, direction, reached_init = false, reached_goal = false, first_time = true;

// Control gain
int D;

// Several variables used in robot motion
float desiredVel, xInit, yInit, zInit, dirX, dirY, theta;

// Starting time of robot motion
double start_time;

// Goal positions
std::vector<float> xGoal, yGoal;

// ROS parameters
ros::Publisher cmd_vel_pub, time_pub, reset_game_pub;
geometry_msgs::PointStamped current_pos, init_pos, goal_pos;
geometry_msgs::Twist cmd_vel;
