// 仿真用 激光扫描代码有所不同
// 用来导出数据
#include "vfhdriver.h"
#include "vfh_algorithm.h"
#include "gnuplot.h"
#include "text/to_string.h"
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
#include <sstream>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ctime>
#include <carmen.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <string.h>
#include <sstream>

std::string robot1_closest_obs_file("/home/gglin/WORKSPACE/exp3_ws/data/robot1_closest_obs.txt");
std::string robot2_closest_obs_file("/home/gglin/WORKSPACE/exp3_ws/data/robot2_closest_obs.txt");
// std::string robot1_v_cos ("/home/gglin/WORKSPACE/exp3_ws/data/robot1_v_cos.txt");
// std::string robot2_v_cos ("/home/gglin/WORKSPACE/exp3_ws/data/robot1_v_cos.txt");

class RobotPose
{
public:
  RobotPose() {}
  double x;
  double y;
  double yaw;
  double timestamp;
};

std::vector<double> v_laserScan2;

std::vector<RobotPose> v_robot1_pose;
std::vector<nav_msgs::Odometry> v_robot1_odom;
std::vector<RobotPose> v_robot2_pose;
std::vector<nav_msgs::Odometry> v_robot2_odom;

// robot1_surrounding true代表有障碍
bool robot1_surrounding = false;

// 指robot1_nearest_obs2 是否准备好了 robot1scan -> robot1odom -> robot1_nearest_obs2
bool robot1_ready = false;

RobotPose robot1_nearest_obs;
RobotPose robot1_nearest_obs2;
RobotPose robot2_nearest_obs;

double heading = 0;

geometry_msgs::Twist currentVel;
geometry_msgs::Twist vel_msg;

vfh::VfhDriver *vfhDriver = new vfh::VfhDriver("/home/gglin/WORKSPACE/human_following/src/follow_control/config/robot2VFHConfig.txt");
ros::Publisher robot_pub;

void robot1ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // std::cout << "robot1ScanCallback in " << std::endl;

  const double LIDAR_ERR = 0.05;
  const double LIDAR_MAX = 30;
  double min_dis = 100;
  double min_ang = 0;
  RobotPose min_point;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->ranges[i] >= LIDAR_ERR && msg->ranges[i] <= LIDAR_MAX)
    {
      if (min_dis > msg->ranges[i])
      {
        min_dis = msg->ranges[i];
        min_ang = msg->angle_min + i * msg->angle_increment;
      }
    }
  }
  // 判断跟随模式
  // std::cout << "robot1_scan: min_dis "<< min_dis << " min_ang " << min_ang << " angle_min " << msg->angle_min  << " angle_max " << msg->angle_max  << std::endl;

  if (min_dis < 1.85)
  {
    min_point.x = min_dis * cos(min_ang);
    min_point.y = min_dis * sin(min_ang);
    min_point.yaw = 0;
    heading = min_point.y;
    robot1_surrounding = true;
    // std::cout << "robot1_scan: min_point x "<< min_point.x << " min_point y " << min_point.y << " min_point.yaw " << min_point.yaw  << std::endl;
  }
  else
  {
    min_point.x = 0;
    min_point.y = 0;
    robot1_surrounding = false;
  }

  std::ofstream robot1_closest_obs(robot1_closest_obs_file, std::ios::app);
  // time x y
  robot1_closest_obs << std::to_string(msg->header.stamp.toSec()) << " \t " << std::to_string(min_point.x) << " \t " << std::to_string(min_point.y) << "\n";
  robot1_closest_obs.close();
}

void robot2ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // std::cout << "robot2ScanCallback in " << std::endl;
  v_laserScan2.clear();
  const double LIDAR_ERR = 0.10;
  const double LIDAR_MAX = 30;
  double min_dis = 3;
  double min_ang = 0;
  RobotPose min_point;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (i % 2 == 0)
      v_laserScan2.push_back(msg->ranges[i]);
    if (msg->ranges[i] >= LIDAR_ERR && msg->ranges[i] <= LIDAR_MAX)
    {
      if (min_dis > msg->ranges[i])
      {
        if (i > 600 || i < 120)
        {
          min_dis = msg->ranges[i];
          min_ang = msg->angle_min + i * msg->angle_increment;
        }
      }
    }
  }
  if (min_dis < 1.85)
  {
    min_point.x = min_dis * cos(min_ang);
    min_point.y = min_dis * sin(min_ang);
    min_point.yaw = 0;
    // std::cout << "robot1_scan: min_point x "<< min_point.x << " min_point y " << min_point.y << " min_point.yaw " << min_point.yaw  << std::endl;
  }
  else
  {
    min_point.x = 0;
    min_point.y = 0;
    robot1_surrounding = false;
  }
  std::ofstream robot2_closest_obs(robot2_closest_obs_file, std::ios::app);
  // time x y
  robot2_closest_obs << std::to_string(msg->header.stamp.toSec()) << " \t " << std::to_string(min_point.x) << " \t " << std::to_string(min_point.y) << "\n";
  robot2_closest_obs.close();

  v_laserScan2.push_back(msg->ranges.back());
}

void robot1PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  // std::cout << "robot1PoseCallback in " << std::endl;
  double timestamp = msg->header.stamp.toSec();
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yaw;

  nav_msgs::Odometry robot1_odom = *msg;
  v_robot1_pose.push_back(pose);
  v_robot1_odom.push_back(robot1_odom);
  if (robot1_surrounding)
  {
    RobotPose obs_loc_deviation;
    obs_loc_deviation.x = robot1_nearest_obs.y * sin(yaw) - robot1_nearest_obs.x * cos(yaw);
    obs_loc_deviation.y = robot1_nearest_obs.y * cos(yaw) - robot1_nearest_obs.x * sin(yaw);
    obs_loc_deviation.yaw = 0;
    robot1_nearest_obs2 = obs_loc_deviation;

    robot1_ready = true;
    // std::cout << "robot1_pose: robot1_nearest_obs2 x:" << robot1_nearest_obs2.x << " y:" << robot1_nearest_obs2.y << std::endl;
  }
  else
  {
    robot1_ready = false;
  }
}

void robot2PoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double timestamp = msg->header.stamp.toSec();

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose robot2_pose;
  robot2_pose.timestamp = timestamp;
  robot2_pose.x = msg->pose.pose.position.x;
  robot2_pose.y = msg->pose.pose.position.y;
  robot2_pose.yaw = yaw;
  v_robot2_pose.push_back(robot2_pose);

  nav_msgs::Odometry robot2_odom = *msg;

  if (!v_robot1_odom.empty())
  {
    // 计算相对位置
    RobotPose robot1_pose;
    nav_msgs::Odometry robot1_odom = v_robot1_odom.back();

    tf::Pose robot1_tf_pose;
    robot1_tf_pose.setOrigin(tf::Vector3(robot1_odom.pose.pose.position.x, robot1_odom.pose.pose.position.y, robot1_odom.pose.pose.position.z));
    tf::Quaternion robot1_tf_q;
    robot1_tf_q.setW(robot1_odom.pose.pose.orientation.w);
    robot1_tf_q.setX(robot1_odom.pose.pose.orientation.x);
    robot1_tf_q.setY(robot1_odom.pose.pose.orientation.y);
    robot1_tf_q.setZ(robot1_odom.pose.pose.orientation.z);
    robot1_tf_pose.setRotation(robot1_tf_q);

    tf::Pose robot2_tf_pose;
    robot2_tf_pose.setOrigin(tf::Vector3(robot2_odom.pose.pose.position.x, robot2_odom.pose.pose.position.y, robot2_odom.pose.pose.position.z));
    tf::Quaternion robot2_tf_q;
    robot2_tf_q.setW(robot2_odom.pose.pose.orientation.w);
    robot2_tf_q.setX(robot2_odom.pose.pose.orientation.x);
    robot2_tf_q.setY(robot2_odom.pose.pose.orientation.y);
    robot2_tf_q.setZ(robot2_odom.pose.pose.orientation.z);
    robot2_tf_pose.setRotation(robot2_tf_q);

    tf::Pose relative_pose;
    relative_pose = robot2_tf_pose.inverse() * robot1_tf_pose;
    tf::Quaternion relative_q = relative_pose.getRotation();

    double relative_roll, relative_pitch, relative_yaw;
    tf::Matrix3x3(relative_q).getRPY(relative_roll, relative_pitch, relative_yaw);
    tf::Vector3 relative_pose_v = relative_pose.getOrigin();

    double relative_x, relative_y, relative_z;
    relative_x = relative_pose_v.x();
    relative_y = relative_pose_v.y();

    std::cout << "robot2_pose: relative_pose x:" << relative_x << " y:" << relative_y << std::endl;
    std::cout << "robot2_pose: robot1_surrounding:" << robot1_surrounding << std::endl;

    geometry_msgs::Twist goal;

    // //计算目标周围是否有障碍
    // if(robot1_surrounding) //有障碍
    // {
    //   //relative_x = relative_x + 0.5 * robot1_nearest_obs2.x;
    //   relative_y = relative_y + 0.5 * heading;

    //   goal.linear.x = sqrt((relative_x * relative_x) + (relative_y * relative_y));
    //   goal.angular.z = atan2(relative_y, relative_x);
    // }
    // else
    // {
    goal.linear.x = sqrt((relative_x * relative_x) + (relative_y * relative_y));
    goal.angular.z = atan2(relative_y, relative_x);
    // }

    if (!v_laserScan2.empty())
    {
      vel_msg = vfhDriver->approachGoalCommand(0.1, goal, currentVel, v_laserScan2);
      double PI = goal.linear.x / 0.5;
      // if(PI > 1) PI = 1;
      // vel_msg.linear.x  = PI*vel_msg.linear.x;
      // vel_msg.angular.z = PI*vel_msg.angular.z;

      std::cout << "robot2_pose: goal relative_pose x:" << relative_x << " y:" << relative_y << std::endl;
      // robot_pub.publish(vel_msg);
    }
  }

  robot1_ready = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_control");
  ros::NodeHandle nh;

  currentVel.linear.x = 0;
  currentVel.angular.z = 0;
  robot_pub = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1);

  ros::Subscriber sub_robot1_pose = nh.subscribe("/robot1/odom", 1000, robot1PoseCallback);
  ros::Subscriber sub_robot2_pose = nh.subscribe("/robot2/odom", 1000, robot2PoseCallback);

  ros::Subscriber sub_robot1_scan = nh.subscribe("/robot1/scan", 1000, robot1ScanCallback);
  ros::Subscriber sub_robot2_scan = nh.subscribe("/robot2/scan", 1000, robot2ScanCallback);

  ros::spin();
  return 0;
}
