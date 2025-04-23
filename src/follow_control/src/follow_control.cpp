
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
#include <eigen3/Eigen/Dense>
#include <uwb_lidar_tracking/Target.h>
#include <uwb_lidar_tracking/Targets.h>


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
  bool status;      // for cases the target's probability > 0.5
  bool is_occluded; // to determine the target is occluded or not

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

// for robot smooth control
int control_smooth_filter_size;
boost::circular_buffer<geometry_msgs::Twist> control_msg_buffer(5);
boost::circular_buffer<Goal> goal_buffer(5);

// for obstacle recording
boost::circular_buffer<std::vector<Obstacle>> obstacles_global_buffer;

typedef std::vector<RobotState>::iterator robot_state_it;
typedef std::vector<Obstacle>::iterator obstacle_it;
typedef std::vector<Eval>::iterator evaluation_it;

// for each frame, we should only publish once
ros::Publisher follow_control_pub;

std::vector<double> calVelLimit();
std::vector<double> calAccelLimit(double v, double w);
std::vector<double> calObstacleLimit(RobotState state, std::vector<Obstacle> obstacle);
std::vector<double> calDynamicWindowVel(double v, double w, RobotState state, std::vector<Obstacle> obstacle);
std::vector<RobotState> trajectoryPredict(RobotState state, double v, double w);

const std::string configure_file("/home/com/WORKSPACE/uwb_lidar_following_gglin/src/follow_control/config/dwa_parameters.txt");

RobotState kinematicModel(RobotState state, double control_v, double control_w, double dt);
double getDist(RobotState state, std::vector<Obstacle> obstacle);
double heading(const std::vector<RobotState> &trajectory, Goal goal);
double velocity(const std::vector<RobotState> &trajectory);
double distance(const std::vector<RobotState> &trajectory, const std::vector<Obstacle> &obstacles);
double visibility(const std::vector<RobotState> &trajectory, const Goal &goal, const std::vector<Obstacle> &obstacles, double v, double w);

// get minimum distance from robot(current state) to the obstacles
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
  min_dist = min_dist - radius;
  if (min_dist < 0.0)
  {
    min_dist = 0.01;
  }
  return min_dist;
}

// distance score (obstacles avoidance score)
double distance(const std::vector<RobotState> &trajectory, const std::vector<Obstacle> &obstacles)
{
  if (obstacles.empty())
  {
    return judge_distance;
  }

  double total_min_dist = 0.0;

  for (const auto &state : trajectory)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &obs : obstacles)
    {
      double dist = sqrt((state.x - obs.x) * (state.x - obs.x) + (state.y - obs.y) * (state.y - obs.y));
      if (dist < min_dist)
      {
        min_dist = dist;
      }
    }
    total_min_dist += min_dist;
  }

  double mean_min_dist = total_min_dist / trajectory.size();
  mean_min_dist -= radius;
  if (mean_min_dist < 0.0)
  {
    mean_min_dist = 0.01;
  }
  mean_min_dist = (mean_min_dist / judge_distance) * (mean_min_dist / judge_distance) * mean_min_dist;

  return mean_min_dist;
}

// ensure the robot go to the right direction
double heading(const std::vector<RobotState> &trajectory, Goal goal)
{
  double d_x, d_y;
  d_x = goal.x - trajectory[trajectory.size() - 1].x;
  d_y = goal.y - trajectory[trajectory.size() - 1].y;
  double dist_to_goal = sqrt(pow(d_x, 2) + pow(d_y, 2));
  return exp(0.5 / (dist_to_goal + radius));
}

// faster is preferred
double velocity(const std::vector<RobotState> &trajectory)
{
  return trajectory.back().v;
}

double visibility(const std::vector<RobotState> &trajectory, const Goal &goal, const std::vector<Obstacle> &obstacles, double v, double w)
{
  if (trajectory.empty() || obstacles.empty())
  {
    return 0;
  }

  RobotState robot_temp;

  double dist_obstacle_goal = 100000;
  Obstacle obstacle_temp(100, 100, 100, 100);
  double dist_robot_goal = sqrt((goal.x) * (goal.x) + (goal.y) * (goal.y));

  // 1. find the closet obstacle to the goal
  for (const auto &it_obs : obstacles)
  {
    double dist = sqrt((it_obs.x - goal.x) * (it_obs.x - goal.x) + (it_obs.y - goal.y) * (it_obs.y - goal.y));
    if (dist < dist_obstacle_goal)
    {
      dist_obstacle_goal = dist;
      obstacle_temp = it_obs;
    }
  }
  if (dist_obstacle_goal > 2.5)
  {
    // std::cout << "dist_obstacle_goal: " << dist_obstacle_goal << std::endl;
    return 1;
  }

  double dist_robot_obstacle = 0, dist_robot_obstacle_count = 0;
  for (const auto &state : trajectory)
  {
    double dist = sqrt((state.x - obstacle_temp.x) * (state.x - obstacle_temp.x) + (state.y - obstacle_temp.y) * (state.y - obstacle_temp.y));
    dist_robot_obstacle += dist;
    dist_robot_obstacle_count++;
  }
  dist_robot_obstacle = dist_robot_obstacle / dist_robot_obstacle_count;

  double visibility_eval = 0;

  visibility_eval = exp((dist_obstacle_goal / std::max(dist_robot_obstacle, 0.8)) * (dist_obstacle_goal / std::max(dist_robot_obstacle, 0.8)));

  return visibility_eval;
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

// if the robot is near a obstacles, it should slows down
std::vector<double> calObstacleLimit(RobotState state, std::vector<Obstacle> obstacle)
{
  double v_low = v_min;
  double v_high = getDist(state, obstacle) * 0.5;
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

std::vector<Obstacle> filterObstacles(std::vector<Obstacle> &obstacles, const Goal &goal, double filter_radius)
{
  std::vector<Obstacle> filtered_obstacles;
  for (const auto &obs : obstacles)
  {
    if (std::hypot(goal.x - obs.x, goal.y - obs.y) > filter_radius)
    {
      filtered_obstacles.push_back(obs);
    }
  }
  std::cout << "Number of deleted obstacles: " << obstacles.size()-filtered_obstacles.size() << std::endl;
  return filtered_obstacles;
}

// Calculate optimal v and w, and pushback in the control_msg_buffer
void dynamicWindowApproach(std::vector<Obstacle> temp_obs, const Goal goal)
{
  // interfaces of robot speed, but not necessary
  double v_temp = 0, w_temp = 0;

  geometry_msgs::Twist control_msg;

  // if the target topic is not published anymore
  if (fabs(ros::Time::now().toSec() - last_target_time.toSec()) > 5)
  {
    std::cout << "The target info has not been updated for more than 5 seconds, robot stop!" << std::endl;
    control_msg.linear.x = 0;
    control_msg.angular.z = 0;
    control_msg_buffer.push_back(control_msg);
    return;
  }

  double linear_dist = std::hypot(goal.x, goal.y);
  double linear_speed_gain = linear_dist / 1; // TODO add this "1.5" into configure
  std::cout << "Goal Distance: " << linear_dist << std::endl;

  // TODO add this "1" into configure
  // judge if we are already reach the target
  if (linear_dist < 0.5)
  {
    control_msg.linear.x = 0;
    control_msg.angular.z = 0;
    std::cout << "Reach Target!" << std::endl;
    control_msg_buffer.push_back(control_msg);
    return;
  }

  // start dynamic window approach
  std::vector<Eval> eval_database;
  eval_database.clear();

  double v_optimal = 0;
  double w_optimal = 0;

  RobotState init_robot_state(0, 0, 0, v_temp, w_temp);

  std::vector<Obstacle> t_obs = filterObstacles(temp_obs, goal, 2.5 * radius);

  std::vector<Obstacle> init_obstacle = t_obs; // Filter obstacles within 0.5 meters of the goal


  double dist_robot_nearest_object = 1000, x_tem = 1000, y_tem = 1000;
  for (obstacle_it it_obs = init_obstacle.begin(); it_obs != init_obstacle.end(); it_obs++)
  {
    double dist_robot_object = sqrt((it_obs->x) * (it_obs->x) + (it_obs->y) * (it_obs->y));
    std::cout << " x:" << it_obs->x << " y:" << it_obs->y << " r:" << it_obs->r << " a:" << it_obs->a << std::endl;

    if (dist_robot_nearest_object > dist_robot_object)
    {
      x_tem = it_obs->x;
      y_tem = it_obs->y;
      
      dist_robot_nearest_object = dist_robot_object;
    }
  }
  std::cout << "dist_robot_nearest_object:" << dist_robot_nearest_object << " x"<< x_tem << " y:" <<y_tem  << std::endl;

  std::vector<double> dynamic_window = calDynamicWindowVel(v_temp, w_temp, init_robot_state, init_obstacle);

  std::cout << "Dynamic Window:" << std::endl;
  std::cout << "v min:" << dynamic_window[0] << " \t v max:" << dynamic_window[1] << std::endl;
  std::cout << "w min:" << dynamic_window[2] << " \t w max:" << dynamic_window[3] << std::endl;

  double max_eval = -10000000;
  double max_heading = -100, max_dist = -100, max_vel = -100;
  double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0, sum_visi = 0.0;

  double v = dynamic_window[0];
  int trajectory_count = 0;
  double p_alpha_temp = 0, p_delta_temp = 0, p_beta_temp = 0;
  while (v <= dynamic_window[1] + 0.01)
  {
    double w = dynamic_window[2];
    while (w <= dynamic_window[3] + 0.01)
    {
      std::vector<RobotState> trajectory = trajectoryPredict(init_robot_state, v, w);

      double heading_eval = heading(trajectory, goal);
      double dist_eval = distance(trajectory, init_obstacle);
      double vel_eval = velocity(trajectory);
      double visi_eval = 1;
      if (dist_robot_nearest_object < 2.5 && linear_dist < 2.5)
      {
        visi_eval = visibility(trajectory, goal, init_obstacle, v, w);
      }
      else
      {
        visi_eval = 1;
      }

      sum_vel += vel_eval;
      sum_dist += dist_eval;
      sum_heading += heading_eval;
      sum_visi += visi_eval;
      w += w_sample;
      eval_database.emplace_back(v, w, heading_eval, dist_eval, vel_eval, visi_eval);
      trajectory_count++;
    }
    v += v_sample;
  }
  std::cout << "Evaluating " << trajectory_count << " Trajectories!" << std::endl;

  if (dist_robot_nearest_object < 2.5 && linear_dist < 2.5)
  {
    std::cout << "visibility!" <<std::endl;
    p_alpha_temp = p_alpha;
    p_delta_temp = p_delta;
  }
  else
  {
    std::cout << "following!" <<std::endl;
    p_alpha_temp = 0.4;
    p_delta_temp = 0;
  }

  if(dist_robot_nearest_object < 1.5)
  {
    p_beta_temp = p_beta;
  }
  else
  {
    p_beta_temp = 0;
  }

  if (!eval_database.empty())
  {
    for (evaluation_it eval_it = eval_database.begin(); eval_it != eval_database.end(); eval_it++)
    {
      double eval = (p_alpha_temp * ((eval_it->heading) / sum_heading)) + (p_beta_temp * ((eval_it->dist) / sum_dist)) + (p_gamma * ((eval_it->vel) / sum_vel)) + (p_delta_temp * ((eval_it->visibility) / sum_visi));

      // std::cout << "v " << eval_it->v << " w " << eval_it->w << std::endl;
      // std::cout << "heading " << (p_alpha_temp * ((eval_it->heading) / sum_heading)) << std::endl;
      // std::cout << "dist " << (p_beta_temp * ((eval_it->dist) / sum_dist)) << std::endl;
      // std::cout << "vel " << (p_gamma * ((eval_it->vel) / sum_vel)) << std::endl;
      // std::cout << "visibility " << (p_delta_temp * ((eval_it->visibility) / sum_visi)) << std::endl;

      // std::cout << "eval " << eval << std::endl;
      // std::cout << std::endl;

      if (eval > max_eval)
      {
        max_eval = eval;
        v_optimal = eval_it->v;
        w_optimal = eval_it->w;
        max_heading = (p_alpha * ((eval_it->heading) / sum_heading));
        max_dist = (p_beta * ((eval_it->dist) / sum_dist));
        max_vel = (p_gamma * ((eval_it->vel) / sum_vel));
      }
    }
  }
  else
  {
    std::cout << "There is no trajectory evaluation data, error!!" << std::endl;
  }

  control_msg.linear.x = v_optimal * linear_speed_gain;
  control_msg.linear.x = std::min(control_msg.linear.x, dynamic_window[1]);
  control_msg.angular.z = w_optimal;
  std::cout << "v optimal: " << v_optimal << ";\t w optimal: " << w_optimal << std::endl;

  control_msg_buffer.push_back(control_msg);
}

void followerScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  std::vector<Obstacle> obstacles;
  obstacles.clear();

  for (std::size_t i = 0; i < scan->ranges.size(); ++i)
  {
    double range = scan->ranges[i];
    if (range > 0.2 && range < 10)
    {
      double angle = scan->angle_min + i * scan->angle_increment;
      double x = range * cos(angle);
      double y = range * sin(angle);
      obstacles.push_back(Obstacle(x, y, range, angle));
    }
  }

  obstacles_global_buffer.push_back(obstacles);
}

void targetInfoCallback(const uwb_lidar_tracking::Targets::ConstPtr &msg)
{
  Goal target_goal;

  for (std::size_t i = 0; i < msg->targets.size(); ++i)
  {
    if (msg->targets[i].uwb_id == tracking_uwb_id && msg->targets[i].status) //&& 
    {
      target_goal.status = msg->targets[i].status;
      target_goal.x = msg->targets[i].x;
      target_goal.y = msg->targets[i].y;
      target_goal.is_occluded = msg->targets[i].is_occluded; // seems useless
      goal_buffer.push_back(target_goal);
      last_target_time = ros::Time::now();
    }
  }

  Goal target_goal_mean;
  double x_mean = 0, y_mean = 0, count = 0;
  for (auto it = goal_buffer.begin(); it != goal_buffer.end(); it++)
  {
    x_mean += it->x;
    y_mean += it->y;
    count++;
  }
  target_goal_mean.x = x_mean / count;
  target_goal_mean.y = y_mean / count;

  if (!obstacles_global_buffer.empty())
  {
    dynamicWindowApproach(obstacles_global_buffer.back(), target_goal_mean);
  }

  // slowly and smoothly
  if (!control_msg_buffer.empty())
  {
    double v_acc = 0, w_acc = 0, count_control = 0;
    for (auto it = control_msg_buffer.begin(); it != control_msg_buffer.end(); it++)
    {
      v_acc += it->linear.x;
      w_acc += it->angular.z;
      count_control++;
    }
    geometry_msgs::Twist control_msg_mean;
    control_msg_mean.linear.x = v_acc / count_control;
    control_msg_mean.angular.z = w_acc / count_control;
    follow_control_pub.publish(control_msg_mean);
  }
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
      else if (key == "control_smooth_filter_size")
        control_smooth_filter_size = value;
    }
  }
  file.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_control");
  ros::NodeHandle nh;

  readParametersFromFile(configure_file);

  obstacles_global_buffer.resize(control_smooth_filter_size);
  obstacles_global_buffer.clear();
  control_msg_buffer.resize(control_smooth_filter_size);
  control_msg_buffer.clear();
  goal_buffer.resize(control_smooth_filter_size);
  goal_buffer.clear();

  follow_control_pub = nh.advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 1);

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
