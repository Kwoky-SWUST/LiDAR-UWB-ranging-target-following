#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/circular_buffer.hpp>
#include <cmath>
#include <ctime>
#include <string>
#include <Eigen/Dense>
#include <numeric> // Include this header for std::accumulate and std::inner_product
#include <uwb_lidar_tracking/Target.h>
#include <uwb_lidar_tracking/Targets.h>
#include <visualization_msgs/Marker.h>

std::string target_amcl_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/dataset/record/amcl_record.txt");
std::string robot_amcl_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/dataset/record/robot_amcl.txt");
std::string target_est_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/dataset/record/uwb_record.txt");
std::string target_est_global_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/dataset/record/target_est_global.txt");
std::string error_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/dataset/record/errors.txt");

struct Pose
{
    double x;
    double y;
    double theta;
    double timestamp;
};

std::vector<Pose> target_amcl_data;
std::vector<Pose> robot_amcl_data;
std::vector<Pose> target_est_data;

int mode = -1;
int target_uwb_id = -1;

bool loadConfig()
{
    std::ifstream config_file("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/src/uwb_lidar_tracking/config/error_calculation_mode.txt");
    if (!config_file.is_open())
    {
        ROS_ERROR("Failed to open configuration file");
        return false;
    }

    std::string line;
    while (std::getline(config_file, line))
    {
        if (line.find("mode:") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string key;
            int value;
            if (iss >> key >> value)
            {
                mode = value;
            }
        }
        else if (line.find("target_uwb_id:") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string key;
            int value;
            if (iss >> key >> value)
            {
                target_uwb_id = value;
            }
        }
    }

    std::cout << "mode: " << mode << std::endl;
    std::cout << "target_uwb_id: " << target_uwb_id << std::endl;
    config_file.close();
    return true;
}

void recordTargetAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
    std::cout << "Target AMCL Recording" << std::endl;
    double timestamp = amcl_msg->header.stamp.toSec();
    double x = amcl_msg->pose.pose.position.x;
    double y = amcl_msg->pose.pose.position.y;
    double theta = tf::getYaw(amcl_msg->pose.pose.orientation);
    Pose amcl_pose = {x, y, theta, timestamp};
    target_amcl_data.push_back(amcl_pose);

    std::ofstream amcl_file(target_amcl_file_name, std::ios::app);
    if (amcl_file.is_open())
    {
        amcl_file << std::to_string(timestamp) << " "
                  << x << " "
                  << y << " "
                  << theta << "\n";
        amcl_file.close();
    }
    else
    {
        ROS_ERROR("Unable to open file for writing: %s", target_amcl_file_name.c_str());
    }
}

void recordRobotAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
    std::cout << "Robot AMCL Recording" << std::endl;
    double timestamp = amcl_msg->header.stamp.toSec();
    double x = amcl_msg->pose.pose.position.x;
    double y = amcl_msg->pose.pose.position.y;
    double theta = tf::getYaw(amcl_msg->pose.pose.orientation);
    Pose amcl_pose = {x, y, theta, timestamp};
    robot_amcl_data.push_back(amcl_pose);

    std::ofstream amcl_file(robot_amcl_file_name, std::ios::app);
    if (amcl_file.is_open())
    {
        amcl_file << std::to_string(timestamp) << " "
                  << x << " "
                  << y << " "
                  << theta << "\n";
        amcl_file.close();
    }
    else
    {
        ROS_ERROR("Unable to open file for writing: %s", robot_amcl_file_name.c_str());
    }
}

void recordTargetEst(const uwb_lidar_tracking::Targets::ConstPtr &est_msg)
{
    std::cout << "Robot EST Recording" << std::endl;

    for (const auto &target : est_msg->targets)
    {
        if (target.uwb_id == target_uwb_id || target.status)
        {
            double timestamp = est_msg->header.stamp.toSec();
            Pose est_pose = {target.x, target.y, 0, timestamp};  // No theta in target estimate
            target_est_data.push_back(est_pose);

            std::ofstream est_file(target_est_file_name, std::ios::app);
            if (est_file.is_open())
            {
                est_file << std::to_string(timestamp) << " "
                         << est_pose.x << " "
                         << est_pose.y << "\n";
                est_file.close();
            }
            else
            {
                ROS_ERROR("Unable to open file for writing: %s", target_est_file_name.c_str());
            }
        }
    }
}

Pose findClosestPose(const std::vector<Pose>& poses, double timestamp)
{
    Pose closest_pose;
    double min_diff = std::numeric_limits<double>::max();

    for (const auto& pose : poses)
    {
        double diff = fabs(pose.timestamp - timestamp);
        if (diff < min_diff)
        {
            min_diff = diff;
            closest_pose = pose;
        }
    }
    return closest_pose;
}

void calculateErrorsBetweenAmclAndEst()
{
    std::ifstream target_amcl_file(target_amcl_file_name);
    std::ifstream target_est_file(target_est_file_name);
    if (!target_amcl_file.is_open() || !target_est_file.is_open())
    {
        ROS_ERROR("Unable to open recorded files for reading");
        return;
    }

    target_amcl_data.clear();
    target_est_data.clear();
    double timestamp, x, y, theta, uwb_id;
    while (target_amcl_file >> timestamp >> x >> y)
    {
        target_amcl_data.push_back({x, y, 0, timestamp});
    }
    while (target_est_file >> uwb_id >> timestamp >> x >> y)
    {
        if(x != 0 && y != 0)
        {
          target_est_data.push_back({x, y, 0, timestamp});
        }
    }

    std::ofstream error_file(error_file_name, std::ios::app);
    if (!error_file.is_open())
    {
        ROS_ERROR("Unable to open error file for writing");
        return;
    }

    std::vector<double> error_x_list;
    std::vector<double> error_y_list;
    std::vector<double> error_distance_list;

    for (const auto& amcl_pose : target_amcl_data)
    {
        Pose closest_est_pose = findClosestPose(target_est_data, amcl_pose.timestamp);
        if(fabs(closest_est_pose.timestamp - amcl_pose.timestamp) > 0.1)
        {
            continue;
        }
        double error_x = fabs(amcl_pose.x - closest_est_pose.x);
        double error_y = fabs(amcl_pose.y - closest_est_pose.y);
        double error_distance = sqrt(error_x * error_x + error_y * error_y);

        error_x_list.push_back(error_x);
        error_y_list.push_back(error_y);
        error_distance_list.push_back(error_distance);

        error_file << amcl_pose.timestamp << " "
                   << error_x << " "
                   << error_y << " "
                   << error_distance << "\n";
    }

    target_amcl_file.close();
    target_est_file.close();
    error_file.close();

    // Calculate overall results
    double mean_error_x = std::accumulate(error_x_list.begin(), error_x_list.end(), 0.0) / error_x_list.size();
    double mean_error_y = std::accumulate(error_y_list.begin(), error_y_list.end(), 0.0) / error_y_list.size();
    double mean_error_distance = std::accumulate(error_distance_list.begin(), error_distance_list.end(), 0.0) / error_distance_list.size();

    double sq_sum_x = std::inner_product(error_x_list.begin(), error_x_list.end(), error_x_list.begin(), 0.0);
    double stdev_error_x = std::sqrt(sq_sum_x / error_x_list.size() - mean_error_x * mean_error_x);

    double sq_sum_y = std::inner_product(error_y_list.begin(), error_y_list.end(), error_y_list.begin(), 0.0);
    double stdev_error_y = std::sqrt(sq_sum_y / error_y_list.size() - mean_error_y * mean_error_y);

    double sq_sum_distance = std::inner_product(error_distance_list.begin(), error_distance_list.end(), error_distance_list.begin(), 0.0);
    double stdev_error_distance = std::sqrt(sq_sum_distance / error_distance_list.size() - mean_error_distance * mean_error_distance);

    std::cout << "Overall Results for " << error_x_list.size() << " recordings:" << std::endl;
    std::cout << "Mean Error X: " << mean_error_x << " Stdev: " << stdev_error_x << std::endl;
    std::cout << "Mean Error Y: " << mean_error_y << " Stdev: " << stdev_error_y << std::endl;
    std::cout << "Mean Error Distance: " << mean_error_distance << " Stdev: " << stdev_error_distance << std::endl;
}

Pose calculateGlobalPose(const Pose& robot_pose, const Pose& relative_pose)
{
    Pose global_pose;
    global_pose.x = robot_pose.x + relative_pose.x * cos(robot_pose.theta) - relative_pose.y * sin(robot_pose.theta);
    global_pose.y = robot_pose.y + relative_pose.x * sin(robot_pose.theta) + relative_pose.y * cos(robot_pose.theta);
    global_pose.theta = robot_pose.theta + relative_pose.theta;
    global_pose.timestamp = relative_pose.timestamp;
    return global_pose;
}

void calculateErrorsBetweenTransformAndEst()
{
    std::ifstream target_amcl_file(target_amcl_file_name);
    std::ifstream robot_amcl_file(robot_amcl_file_name);
    std::ifstream target_est_file(target_est_file_name);
    if (!target_amcl_file.is_open() || !robot_amcl_file.is_open() || !target_est_file.is_open())
    {
        ROS_ERROR("Unable to open recorded files for reading");
        return;
    }

    double timestamp, x, y, theta;
    while (target_amcl_file >> timestamp >> x >> y >> theta)
    {
        target_amcl_data.push_back({x, y, theta, timestamp});
    }
    while (robot_amcl_file >> timestamp >> x >> y >> theta)
    {
        robot_amcl_data.push_back({x, y, theta, timestamp});
    }
    while (target_est_file >> timestamp >> x >> y)
    {
        if(x != 0 && y != 0)
        {
          target_est_data.push_back({x, y, 0, timestamp});
        }
    }

    std::ofstream error_file(error_file_name, std::ios::app);
    std::ofstream target_est_global_file(target_est_global_file_name, std::ios::app);
    if (!error_file.is_open() || !target_est_global_file.is_open())
    {
        ROS_ERROR("Unable to open error or global est file for writing");
        return;
    }

    std::vector<double> error_x_list;
    std::vector<double> error_y_list;
    std::vector<double> error_theta_list;
    std::vector<double> error_distance_list;

    for (const auto& robot_pose : robot_amcl_data)
    {
        // Step 2: Find the closest target estimate (B) with matching timestamp
        Pose closest_est_pose = findClosestPose(target_est_data, robot_pose.timestamp);

        // Step 3: Use A (robot_pose) and B (closest_est_pose) to calculate the global pose (C)
        Pose global_target_pose = calculateGlobalPose(robot_pose, closest_est_pose);

        // Step 4: Record C (global_target_pose) and find the closest target AMCL pose (D)
        Pose closest_target_amcl_pose = findClosestPose(target_amcl_data, global_target_pose.timestamp);

        if(fabs(closest_target_amcl_pose.timestamp - robot_pose.timestamp) > 0.1)
        {
            continue;
        }
        // Calculate the errors between C and D
        double error_x = fabs(global_target_pose.x - closest_target_amcl_pose.x);
        double error_y = fabs(global_target_pose.y - closest_target_amcl_pose.y);
        double error_theta = fabs(global_target_pose.theta - closest_target_amcl_pose.theta);
        double error_distance = sqrt(error_x * error_x + error_y * error_y);

        error_x_list.push_back(error_x);
        error_y_list.push_back(error_y);
        error_theta_list.push_back(error_theta);
        error_distance_list.push_back(error_distance);

        error_file << global_target_pose.timestamp << " "
                   << error_x << " "
                   << error_y << " "
                   << error_theta << " "
                   << error_distance << "\n";
        
        target_est_global_file  << global_target_pose.timestamp << " "
                                << global_target_pose.x << " "
                                << global_target_pose.y << " "
                                << global_target_pose.theta << "\n";
    }

    target_amcl_file.close();
    robot_amcl_file.close();
    target_est_file.close();
    error_file.close();
    target_est_global_file.close();

    // Calculate overall results
    double mean_error_x = std::accumulate(error_x_list.begin(), error_x_list.end(), 0.0) / error_x_list.size();
    double mean_error_y = std::accumulate(error_y_list.begin(), error_y_list.end(), 0.0) / error_y_list.size();
    double mean_error_theta = std::accumulate(error_theta_list.begin(), error_theta_list.end(), 0.0) / error_theta_list.size();
    double mean_error_distance = std::accumulate(error_distance_list.begin(), error_distance_list.end(), 0.0) / error_distance_list.size();

    double sq_sum_x = std::inner_product(error_x_list.begin(), error_x_list.end(), error_x_list.begin(), 0.0);
    double stdev_error_x = std::sqrt(sq_sum_x / error_x_list.size() - mean_error_x * mean_error_x);

    double sq_sum_y = std::inner_product(error_y_list.begin(), error_y_list.end(), error_y_list.begin(), 0.0);
    double stdev_error_y = std::sqrt(sq_sum_y / error_y_list.size() - mean_error_y * mean_error_y);

    double sq_sum_theta = std::inner_product(error_theta_list.begin(), error_theta_list.end(), error_theta_list.begin(), 0.0);
    double stdev_error_theta = std::sqrt(sq_sum_theta / error_theta_list.size() - mean_error_theta * mean_error_theta);

    double sq_sum_distance = std::inner_product(error_distance_list.begin(), error_distance_list.end(), error_distance_list.begin(), 0.0);
    double stdev_error_distance = std::sqrt(sq_sum_distance / error_distance_list.size() - mean_error_distance * mean_error_distance);

    std::cout << "Overall Results for Mode 3:" << std::endl;
    std::cout << "Mean Error X: " << mean_error_x << " Stdev: " << stdev_error_x << std::endl;
    std::cout << "Mean Error Y: " << mean_error_y << " Stdev: " << stdev_error_y << std::endl;
    std::cout << "Mean Error Theta: " << mean_error_theta << " Stdev: " << stdev_error_theta << std::endl;
    std::cout << "Mean Error Distance: " << mean_error_distance << " Stdev: " << stdev_error_distance << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "error_calculation");
    ros::NodeHandle nh;

    if (!loadConfig())
    {
        return -1;
    }

    if (mode == 1)
    {
        ros::Subscriber target_amcl_sub = nh.subscribe("/robot2/amcl_pose", 1000, recordTargetAmcl);
        ros::Subscriber robot_amcl_sub = nh.subscribe("/robot1/amcl_pose", 1000, recordRobotAmcl);
        ros::Subscriber target_est_sub = nh.subscribe("/targets", 1000, recordTargetEst);
        ros::spin();
    }
    else if (mode == 2)
    {
        calculateErrorsBetweenAmclAndEst();
    }
    else if (mode == 3)
    {
        calculateErrorsBetweenTransformAndEst();
    }

    return 0;
}
