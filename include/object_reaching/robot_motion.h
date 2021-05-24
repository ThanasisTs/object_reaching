#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <math.h>
#include <vector>

bool prediction = false, direction, reached_init = false, reached_goal = false, first_time = true;
int D;
float desiredVel, xInit, yInit, zInit, dirX, dirY, theta;
double start_time;
std::vector<float> xGoal, yGoal;

ros::Publisher cmd_vel_pub, time_pub, reset_game_pub;
geometry_msgs::PointStamped current_pos, init_pos, goal_pos;
geometry_msgs::Twist cmd_vel;
