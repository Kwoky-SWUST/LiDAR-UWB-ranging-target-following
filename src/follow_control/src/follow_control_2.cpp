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
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <sstream>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include "text/to_string.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ctime>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

std::string lidar_scan_topic="/robot2/scan";
std::string pose1_topic = "/HumanPose";
std::string pose2_topic = "/UAV1Pose";
geometry_msgs::Twist currentVel;
geometry_msgs::Twist vel_msg;
std::vector<double> v_laserScan;
vfh::VfhDriver *vfhDriver = new vfh::VfhDriver("/home/comgroup/WORKSPACE/human_following/src/follow_control/config/robot2VFHConfig.txt");
ros::Publisher robot_pub;

void callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose)
{
    tf::Pose relative_pose;
    tf::Pose tf_human_pose;
    tf_human_pose.setOrigin(tf::Vector3(human_pose->pose.pose.position.x, human_pose->pose.pose.position.y, human_pose->pose.pose.position.z));
    tf::Quaternion tf_human_q;
    tf_human_q.setW(human_pose->pose.pose.orientation.w);
    tf_human_q.setX(human_pose->pose.pose.orientation.x);
    tf_human_q.setY(human_pose->pose.pose.orientation.y);
    tf_human_q.setZ(human_pose->pose.pose.orientation.z);
    tf_human_pose.setRotation(tf_human_q);

    tf::Pose tf_robot_pose;
    tf_robot_pose.setOrigin(tf::Vector3(robot_pose->pose.pose.position.x,robot_pose->pose.pose.position.y,robot_pose->pose.pose.position.z));
    tf::Quaternion tf_robot_q;
    tf_robot_q.setW(robot_pose->pose.pose.orientation.w);
    tf_robot_q.setX(robot_pose->pose.pose.orientation.x);
    tf_robot_q.setY(robot_pose->pose.pose.orientation.y);
    tf_robot_q.setZ(robot_pose->pose.pose.orientation.z);
    tf_robot_pose.setRotation(tf_robot_q);

    relative_pose = tf_robot_pose.inverse() * tf_human_pose;
    tf::Quaternion q = relative_pose.getRotation();

    double relative_roll, relative_pitch, relative_yaw;
    tf::Matrix3x3(q).getRPY(relative_roll, relative_pitch, relative_yaw);
    tf::Vector3 relative_pose_v = relative_pose.getOrigin();
    double range = sqrt(relative_pose_v.x()*relative_pose_v.x() + relative_pose_v.y()*relative_pose_v.y());
    double angle = atan2(relative_pose_v.y(), relative_pose_v.x());
    std::cout << "relative pose: " << relative_pose_v.x() << " " << relative_pose_v.y() << " \n"<< std::endl;
    geometry_msgs::Twist goal;
    goal.linear.x = range;
    goal.angular.z = angle;
    vel_msg = vfhDriver->approachGoalCommand(0.1, goal, currentVel, v_laserScan);
    double PI = range/0.4;
    if(PI > 1) PI = 1;
    vel_msg.linear.x  = PI*vel_msg.linear.x;
    vel_msg.angular.z = PI*vel_msg.angular.z;
    std::cout << "publish: goal range=" << vel_msg.linear.x << " angular=" <<vel_msg.angular.z  <<" \n"<<std::endl;
    robot_pub.publish(vel_msg);
}

void robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    currentVel.linear.x = msg->linear.x;
    currentVel.angular.z = msg->angular.z;
}

void robotScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    v_laserScan.clear();
    const double LIDAR_ERR = 0.10;
    const double LIDAR_MAX = 30;

    //  std::cout << msg->ranges.size() << std::endl;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        if (i % 2 == 0)
            v_laserScan.push_back(msg->ranges[i]);
    }
    v_laserScan.push_back(msg->ranges.back());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_control");
    ros::NodeHandle nh;

    ros::Subscriber sub_robot_vel = nh.subscribe("/robot2/cmd_vel_mux/input/teleop", 1000, robotCmdVelCallback);    
    ros::Subscriber sub_robot_scan = nh.subscribe("/robot2/scan", 1000, robotScanCallback);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, lidar_scan_topic, 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose1_sub(nh, pose1_topic, 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose2_sub(nh, pose2_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> controlSyncPolicy;
    message_filters::Synchronizer<controlSyncPolicy> sync(controlSyncPolicy(30), scan_sub, pose1_sub, pose2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    robot_pub = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel_mux/input/teleop", 1);
    
    ros::spin();
    return 0;
}