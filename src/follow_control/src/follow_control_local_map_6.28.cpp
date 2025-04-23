
#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <ctime>
#include <stdlib.h>
#include <string.h>
#include <Eigen/Dense>
#include "Target.h"
#include "Targets.h"

double dt;                                // 采样时间
double v_min, v_max, w_min, w_max;        // 线速度角速度边界
double predict_time;                      // 轨迹推算时间长度
double a_vmax, a_wmax;                    // 线加速度和角加速度最大值
double v_sample, w_sample;                // 采样分辨率
double p_alpha, p_beta, p_gamma, p_delta; // 轨迹评价函数系数
double radius;                            // 用于判断是否到达目标点
double judge_distance;                    // 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
int tracking_uwb_id;                      // 跟随的UWB 的id

ros::Time last_target_time;

struct RobotState
{
  double x;
  double y;
  double theta;
  double v;
  double w;

  RobotState(double x, double y, double theta, double v, double w)
      : x(x), y(y), theta(theta), v(v), w(w) {}
  RobotState()
  {
    x = 0;
    y = 0;
    theta = 0;
    v = 0;
    w = 0;
  }
};

struct Obstacle
{
  double x;
  double y;
  double r; // range
  double a; // angle

  Obstacle(double x, double y, double r, double a) : x(x), y(y), r(r), a(a) {}
  Obstacle()
  {
    x = 0;
    y = 0;
    r = 0;
    a = 0;
  }
};

struct Goal
{
  double x;
  double y;
  bool status;

  Goal()
  {
    x = 0;
    y = 0;
    status = false;
  }
  Goal(double x, double y, bool status) : x(x), y(y), status(false) {}
};

struct Eval
{
  double v;
  double w;
  double heading;
  double dist;
  double vel;
  double visibility;
  double _min_distance;

  Eval(double v, double w, double heading, double dist, double vel, double visibility) : v(v), w(w), heading(heading), dist(dist), vel(vel), visibility(visibility) {}
  Eval()
  {
    v = 0;
    w = 0;
    heading = 0;
    dist = 0;
    vel = 0;
    visibility = 0;
    _min_distance = 0;
  }
};

struct Control
{
  double v;
  double w;

  Control(double v, double w) : v(v), w(w) {}
  Control()
  {
    v = 0;
    w = 0;
  }
};

boost::circular_buffer<Control> control_msg_buffer(10);

Goal goal_global(0, 0, false);
boost::circular_buffer<std::vector<Obstacle>> obstacles_global_buffer;
typedef std::vector<RobotState>::iterator robot_state_it;
typedef std::vector<Obstacle>::iterator obstacle_it;
typedef std::vector<Eval>::iterator evaluation_it;

geometry_msgs::Twist current_vel;
nav_msgs::OccupancyGrid local_map;
ros::Publisher follow_control_pub;
ros::Publisher fake_laser_pub;
ros::Publisher local_map_pub;

std::vector<double> calVelLimit();
std::vector<double> calAccelLimit(double v, double w);
std::vector<double> calObstacleLimit(RobotState state, std::vector<Obstacle> obstacle);
std::vector<double> calDynamicWindowVel(double v, double w, RobotState state, std::vector<Obstacle> obstacle);
std::vector<RobotState> trajectoryPredict(RobotState state, double v, double w);
void dynamicWindowApproach(std::vector<Obstacle> t_obs);

const std::string configure_file("/home/gglin/WORKSPACE/uwb_lidar_following/src/follow_control/config/dwa_parameters.txt");

RobotState kinematicModel(RobotState state, double control_v, double control_w, double dt);
double getDist(RobotState state, std::vector<Obstacle> obstacle);
double heading(std::vector<RobotState> trajectory, Goal goal);
double velocity(std::vector<RobotState> trajectory);
double distance(std::vector<RobotState> trajectory, std::vector<Obstacle> obstacle);
double visibility(std::vector<RobotState> trajectory, Goal goal, std::vector<Obstacle> obstacles, double v, double w);

// get minimum distance from robot to the obstacles
double getDist(RobotState state, std::vector<Obstacle> obstacle)
{
  std::vector<Obstacle> t_obstacles = obstacles_global_buffer.back();
  if (t_obstacles.empty())
  {
    return judge_distance + 5.0;
  }
  double min_dist = 100000;

  for (obstacle_it it_obs = t_obstacles.begin(); it_obs != t_obstacles.end(); it_obs++)
  {
    double dist = sqrt((it_obs->x - state.x) * (it_obs->x - state.x) + (it_obs->y - state.y) * (it_obs->y - state.y));
    min_dist = std::min(min_dist, dist);
  }
  min_dist = min_dist - 0.3;
  if (min_dist < 0.0)
  {
    min_dist = 0.01;
  }
  std::cout << "min_dist " << min_dist << std::endl;
  return min_dist;
}

// distance score
double distance(std::vector<RobotState> trajectory, std::vector<Obstacle> obstacle)
{
  if (obstacle.empty())
  {
    return judge_distance;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (obstacle_it it_obs = obstacle.begin(); it_obs != obstacle.end(); it_obs++)
  {
    for (robot_state_it it_robot = trajectory.begin(); it_robot != trajectory.end(); it_robot++)
    {
      double dist = sqrt((it_robot->x - it_obs->x) * (it_robot->x - it_obs->x) + (it_robot->y - it_obs->y) * (it_robot->y - it_obs->y));

      min_dist = std::min(min_dist, dist);
    }
  }
  min_dist = min_dist - 0.3;  // inflation 0.3
  if (min_dist < 0.0)
  {
    min_dist = 0.01;
  }
  min_dist = (min_dist / judge_distance) * (min_dist / judge_distance) * min_dist;

  return min_dist;
}

double heading(std::vector<RobotState> trajectory, Goal goal)
{
  double d_x, d_y;
  d_x = goal.x - trajectory[trajectory.size() - 1].x;
  d_y = goal.y - trajectory[trajectory.size() - 1].y;
  double dist_to_goal = sqrt(pow(d_x, 2) + pow(d_y, 2));
  // double heading_error = atan2(d_y, d_x) - trajectory[trajectory.size() - 1].theta;
  return exp(-dist_to_goal);
}

double velocity(std::vector<RobotState> trajectory)
{
  return trajectory.back().v;
}

std::vector<double> calVelLimit()
{
  return {v_min, v_max, w_min, w_max};
}

std::vector<double> calAccelLimit(double v, double w)
{
  double v_low = v - a_vmax * dt;
  double v_high = v + a_vmax * dt;
  double w_low = w - a_wmax * dt;
  double w_high = w + a_wmax * dt;
  return {v_low, v_high, w_low, w_high};
}

std::vector<double> calObstacleLimit(RobotState state, std::vector<Obstacle> obstacle)
{
  double v_low = v_min;
  double v_high = getDist(state, obstacle) * 0.5;
  std::cout << "v_high = " << v_high <<std::endl;
  double w_low = w_min;
  double w_high = w_max;
  return {v_low, v_high, w_low, w_high};
}

std::vector<double> calDynamicWindowVel(double v, double w, RobotState state, std::vector<Obstacle> obstacle)
{
  std::vector<double> Vm = calVelLimit();
  std::vector<double> Vd = calAccelLimit(v, w);
  std::vector<double> Va = calObstacleLimit(state, obstacle);
  double a = std::max({Vm[0], Vd[0], Va[0]});
  double b = std::min({Vm[1], Vd[1], Va[1]});
  double c = std::max({Vm[2], Vd[2], Va[2]});
  double d = std::min({Vm[3], Vd[3], Va[3]});
  return {a, b, c, d};
}

RobotState kinematicModel(RobotState state, double control_v, double control_w, double dt)
{
  double x_new = state.x + control_v * cos(state.theta) * dt;
  double y_new = state.y + control_v * sin(state.theta) * dt;
  double theta_new = state.theta + control_w * dt;
  return RobotState(x_new, y_new, theta_new, control_v, control_w);
}

std::vector<RobotState> trajectoryPredict(RobotState state, double v, double w)
{
  std::vector<RobotState> trajectory;
  trajectory.push_back(state);
  double time = 0;
  while (time <= predict_time)
  {
    state = kinematicModel(state, v, w, dt);
    trajectory.push_back(state);
    time += dt;
  }
  return trajectory;
}

void dynamicWindowApproach(std::vector<Obstacle> temp_obs)
{
  double v_temp = 0;
  double w_temp = 0;

  std::vector<Obstacle> t_obs = temp_obs;

  geometry_msgs::Twist control_msg;

  if (fabs(ros::Time::now().toSec() - last_target_time.toSec()) > 5)
  {
    std::cout << "The target info has not been updated for more than 5 seconds, robot stop!" << std::endl;
    control_msg.linear.x = 0;
    control_msg.angular.z = 0;
    follow_control_pub.publish(control_msg);
    return;
  }

  double linear_dist = sqrt(goal_global.x * goal_global.x + goal_global.y * goal_global.y);
  double linear_speed_gain = linear_dist / 1;

  if (linear_dist < 0.5)
  {
    control_msg.linear.x = 0;
    control_msg.angular.z = 0;
    follow_control_pub.publish(control_msg);
    std::cout << "Reach Target!" << std::endl;
    return;
  }

  // start dynamic window approach
  std::vector<Eval> eval_database;
  eval_database.clear();

  double v_optimal = 0;
  double w_optimal = 0;

  // get the obstacle position from obstacles_global
  // initiate the robot pose as 0,0,0
  // get the goal position from goal_global
  // calculate the dynamic window
  // calculate the trajectory
  // evaluate the trajectory
  // select the best trajectory
  RobotState init_robot_state(0, 0, 0, v_temp, w_temp);

  std::vector<Obstacle> init_obstacle = t_obs;

  std::vector<double> dynamic_window = calDynamicWindowVel(v_temp, w_temp, init_robot_state, init_obstacle);

  std::cout << "dynamic window:" << std::endl;
  std::cout << dynamic_window[0] << std::endl;
  std::cout << dynamic_window[1] << std::endl;
  std::cout << dynamic_window[2] << std::endl;
  std::cout << dynamic_window[3] << std::endl;

  double max_eval = -10000000;

  double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0, sum_visi = 0.0;

  double v = dynamic_window[0];
  while (v <= dynamic_window[1] + 0.01)
  {
    double w = dynamic_window[2];
    while (w <= dynamic_window[3] + 0.01)
    {
      std::vector<RobotState> trajectory = trajectoryPredict(init_robot_state, v, w);

      double heading_eval = heading(trajectory, goal_global);
      double dist_eval = distance(trajectory, init_obstacle);
      // double dist_eval = 1;
      double vel_eval = velocity(trajectory);
      double visi_eval = 1;

      sum_vel += vel_eval;
      sum_dist += dist_eval;
      sum_heading += heading_eval;
      sum_visi += visi_eval;
      w += w_sample;
      eval_database.push_back(Eval(v, w, heading_eval, dist_eval, vel_eval, visi_eval));
    }
    v += v_sample;
  }

  // first check eval_database is not empty
  // second find the max eval of different v w samples
  // third publish the v w optimal
  // fourth clear the eval_database

  if (!eval_database.empty())
  {
    for (evaluation_it eval_it = eval_database.begin(); eval_it != eval_database.end(); eval_it++)
    {
      double eval = (p_alpha * ((eval_it->heading) / sum_heading)) + (p_beta * ((eval_it->dist) / sum_dist)) + (p_gamma * ((eval_it->vel) / sum_vel)) + (p_delta * ((eval_it->visibility) / sum_visi));

      if (eval > max_eval)
      {
        max_eval = eval;
        v_optimal = eval_it->v;
        w_optimal = eval_it->w;
      }
    }
  }
  else
  {
    std::cout << "there is no trajectory evaluation data, error!!" << std::endl;
  }
  control_msg.linear.x = v_optimal * linear_speed_gain;
  control_msg.linear.x = std::min(control_msg.linear.x, dynamic_window[1]);
  control_msg.angular.z = w_optimal;
  std::cout << "v_optimal: " << v_optimal << ";\t w_optimal: " << w_optimal << std::endl;
  follow_control_pub.publish(control_msg);
  eval_database.clear();
  goal_global.status = false;
}

void followerScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  // handle the laser data and obtain the obstalces' positions using a local_map

  if (scan->header.stamp.toSec() - local_map.header.stamp.toSec() < 0.3) // update local_map every 1 second
  {
    return;
  }

  local_map.header.stamp = scan->header.stamp;
  local_map.info.resolution = 0.3;
  local_map.info.width = 40;
  local_map.info.height = 60;
  local_map.info.origin.position.x = 0; // Centered at the robot's origin
  local_map.info.origin.position.y = 0;
  local_map.data.resize(local_map.info.width * local_map.info.height);
  std::fill(local_map.data.begin(), local_map.data.end(), 100);

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {

    double range;
    if (scan->ranges[i] > scan->range_min && scan->ranges[i] < local_map.info.resolution * local_map.info.width)
    {
      range = scan->ranges[i];
    }
    else
    {
      // using the longest ranging to fill the map
      range = local_map.info.resolution * local_map.info.width;
    }

    double angle = scan->angle_min + i * scan->angle_increment;

    // Convert polar coordinates to Cartesian
    double x = range * cos(angle);
    double y = range * sin(angle);

    // Convert to map coordinates
    int map_x = (x - local_map.info.origin.position.x) / local_map.info.resolution;
    int map_y = (y - local_map.info.origin.position.y) / local_map.info.resolution;

    if (map_x >= 0 && map_x < local_map.info.width && map_y >= (-0.5) * local_map.info.height && map_y < 0.5 * local_map.info.height)
    {
      // Mark the cells along the beam as free
      for (double r = 0; r < range; r += local_map.info.resolution)
      {
        double free_x = r * cos(angle);
        double free_y = r * sin(angle);
        int free_map_x = (free_x - local_map.info.origin.position.x) / local_map.info.resolution;
        int free_map_y = (free_y - local_map.info.origin.position.y) / local_map.info.resolution;

        if (free_map_x >= 0 && free_map_x < local_map.info.width && free_map_y >= (-0.5) * local_map.info.height && free_map_y < 0.5 * local_map.info.height)
        {
          int free_index = (free_map_y + (0.5) * local_map.info.height) * local_map.info.width + free_map_x;
          local_map.data[free_index] = 0; // Mark as free
        }
      }
      int map_index = (map_x + (0.5) * local_map.info.height) * local_map.info.width + map_y;
    }
  }

  local_map_pub.publish(local_map);
}

void targetInfoCallback(const uwb_lidar_tracking::Targets::ConstPtr &msg)
{
  // use the local map and the target info,
  // to get the obstacles' positions in the local map (and remove the target position from occupied grids)
  last_target_time = ros::Time::now();
  std::vector<Obstacle> t_obstacles;
  t_obstacles.clear();

  for (size_t i = 0; i < msg->targets.size(); ++i)
  {
    if (msg->targets[i].uwb_id == tracking_uwb_id)
    {
      goal_global.status = msg->targets[i].status;
      goal_global.x = msg->targets[i].x;
      goal_global.y = msg->targets[i].y;
    }
  }

  nav_msgs::OccupancyGrid t_local_map = local_map;
  double map_offset_x = 0;
  double map_offset_y = (-0.5) * t_local_map.info.height;

  // get the obstacles position
  for (size_t i = 0; i < t_local_map.data.size(); ++i)
  {
    if (t_local_map.data[i] == 100)
    { // Check if the cell is occupied
      int map_y = i / t_local_map.info.width;
      int map_x = i % t_local_map.info.width;

      // Convert to world coordinates
      double world_x = (map_x + map_offset_x) * t_local_map.info.resolution + t_local_map.info.origin.position.x;
      double world_y = (map_y + map_offset_y) * t_local_map.info.resolution + t_local_map.info.origin.position.y;

      Obstacle temp_obs(world_x, world_y, atan2(world_y, world_x), sqrt(world_x * world_x + world_y * world_y));
      t_obstacles.push_back(temp_obs);
    }
  }

  obstacles_global_buffer.push_back(t_obstacles);
  dynamicWindowApproach(t_obstacles);
  t_obstacles.clear();
}

// 从文件读取参数的函数
void readParametersFromFile(const std::string &filename)
{
  std::ifstream file(filename);

  if (!file.is_open())
  {
    std::cerr << "Error: Unable to open file " << filename << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue; // Skip empty lines and comments

    std::istringstream iss(line);
    std::string key, value_str;
    double value;
    if (std::getline(iss, key, ':') && std::getline(iss, value_str))
    {
      std::istringstream(value_str) >> value;

      if (key == "dt")
        dt = value;
      else if (key == "v_min")
        v_min = value;
      else if (key == "v_max")
        v_max = value;
      else if (key == "w_min")
        w_min = value;
      else if (key == "w_max")
        w_max = value;
      else if (key == "predict_time")
        predict_time = value;
      else if (key == "a_vmax")
        a_vmax = value;
      else if (key == "a_wmax")
        a_wmax = value;
      else if (key == "v_sample")
        v_sample = value;
      else if (key == "w_sample")
        w_sample = value;
      else if (key == "alpha")
        p_alpha = value;
      else if (key == "beta")
        p_beta = value;
      else if (key == "gamma")
        p_gamma = value;
      else if (key == "delta")
        p_delta = value;
      else if (key == "radius")
        radius = value;
      else if (key == "judge_distance")
        judge_distance = value;
      else if (key == "tracking_uwb_id")
        tracking_uwb_id = value;
    }
  }
  file.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_control");
  ros::NodeHandle nh;

  readParametersFromFile(configure_file);

  obstacles_global_buffer.resize(3);
  obstacles_global_buffer.clear();
  control_msg_buffer.resize(10);
  control_msg_buffer.clear();

  follow_control_pub = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1);
  // fake_laser_pub = nh.advertise<sensor_msgs::LaserScan>("/fake_laser", 1);
  local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

  ros::Subscriber sub_follow_scan = nh.subscribe("/robot2/scan", 1000, followerScanCallback);

  ros::Subscriber sub_follow_goal = nh.subscribe("/targets", 1000, targetInfoCallback);

  std::cout << "Dynamic Window Approach Parameters:" << dt << std::endl;
  std::cout << "dt: \t\t\t" << dt << std::endl;
  std::cout << "p_alpha: \t\t" << p_alpha << std::endl;
  std::cout << "p_beta: \t\t" << p_beta << std::endl;
  std::cout << "p_gamma: \t\t" << p_gamma << std::endl;
  std::cout << "p_delta: \t\t" << p_delta << std::endl;
  std::cout << "predict_time: \t\t" << predict_time << std::endl;
  std::cout << "v_min: \t\t\t" << v_min << std::endl;
  std::cout << "v_max: \t\t\t" << v_max << std::endl;
  std::cout << "w_min: \t\t\t" << w_min << std::endl;
  std::cout << "w_max: \t\t\t" << w_max << std::endl;
  std::cout << "a_vmax: \t\t" << a_vmax << std::endl;
  std::cout << "a_wmax: \t\t" << a_wmax << std::endl;
  std::cout << "v_sample: \t\t" << v_sample << std::endl;
  std::cout << "w_sample: \t\t" << w_sample << std::endl;
  std::cout << "radius: \t\t" << radius << std::endl;
  std::cout << "judge_distance: \t" << judge_distance << std::endl;
  std::cout << "tracking uwb id: \t" << tracking_uwb_id << std::endl;

  ros::spin();
  return 0;
}
