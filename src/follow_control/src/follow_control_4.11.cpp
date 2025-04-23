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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ctime>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <livox_ros_driver/CustomMsg.h>
#include <carmen.h>

int time1 = 0;
int time2 = 0;
int time3 = 0;
std::string lidar_scan_topic="/robot2/scan";
std::string pose1_topic = "/HumanPose";
std::string pose2_topic = "/UAV1Pose";
geometry_msgs::Twist currentVel;

geometry_msgs::Twist vel_msg;
std::vector<double> v_laserScan;
vfh::VfhDriver *vfhDriver = new vfh::VfhDriver("/home/comgroup/WORKSPACE/human_following/src/follow_control/config/robot2VFHConfig.txt");
ros::Publisher robot_pub;
//GnuplotInterface *plot_2d_lidar = new GnuplotInterface();
std::vector<geometry_msgs::Twist> control_msg_buffer;

void robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    //std::cout<< "robotCmdVelCallback in" << std::endl;
    currentVel.linear.x = msg->linear.x;
    currentVel.angular.z = msg->angular.z; 
    //time3++;
}

void callback_relative(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose)
{
    //std::cout << "callback_relative in!" << std::endl;
    if(!v_laserScan.empty())
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
        double range = sqrt(relative_pose_v.x() * relative_pose_v.x() + relative_pose_v.y() * relative_pose_v.y());
        double angle = atan2(relative_pose_v.y(), relative_pose_v.x());
        std::cout << "relative pose x y angle: " << relative_pose_v.x() << " " << relative_pose_v.y() << " " << angle * (180.0/ M_PI)
                  << std::endl;
        geometry_msgs::Twist goal;
        goal.linear.x = range;
        goal.angular.z = angle;
        vel_msg = vfhDriver->approachGoalCommand(0.1, goal, currentVel, v_laserScan);
        double PI = range/0.4;
        if(PI > 1) PI = 1;
        vel_msg.linear.x  = PI*vel_msg.linear.x;
        vel_msg.angular.z = PI*vel_msg.angular.z;
        control_msg_buffer.push_back(vel_msg);
        std::cout << "publish: control command = " << vel_msg.linear.x << " angular = " << vel_msg.angular.z << " in degree: " << vel_msg.angular.z * (180.0 / M_PI) << " + 90 " 
        << vel_msg.angular.z * (180.0 / M_PI) + 90.0 << " control_msg_buffer size " << control_msg_buffer.size() << std::endl;
    }
}

void callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    //v_t_laserScan.clear();

    std::vector<livox_ros_driver::CustomPoint> points_3d = msg->points;
    std::cout << "livox lidar point size = \t" << points_3d.size() << std::endl;
    carmen_point_t points_2d_3[361];
    std::vector<double> ranges_degree(361);
    std::vector<std::vector<carmen_point_t>> point_2d_2(361);

    for(int i = 0; i < points_3d.size(); i ++)
    {
        if(-0.01 < points_3d[i].z && points_3d[i].z < 0.01)
        {
            double theta = atan2(points_3d[i].y,points_3d[i].x);
            theta = ((double)180.0)/M_PI * theta;
            theta += 90.0; 
            int index = floor(theta);
            carmen_point_t point_temp;
            point_temp.x = points_3d[i].x;;
            point_temp.y = points_3d[i].y;
            point_temp.theta = theta;
            point_2d_2[index].push_back(point_temp);
        }
    }

    for(int i = 0; i < 361 ; i ++)
    {
        carmen_point_t point_mean;
        int index=0;
        double x=0,y=0;
        for(int j=0; j< point_2d_2[i].size();j++)
        {
            if(point_2d_2[i][j].x > 0 || point_2d_2[i][j].y >0)
            {
                x = x + point_2d_2[i][j].x;
                y = y + point_2d_2[i][j].y;
                index ++;
            }
        }
        if(index > 0)
        {
            point_mean.x = x/(double)index;
            point_mean.y = y/(double)index;
        }
        else
        {
            point_mean.x = INFINITY;
            point_mean.y = INFINITY; 
        }
        point_2d_2[i].push_back(point_mean);
        points_2d_3[i] = point_mean;
    }

    for(int i = 1; i <= 180 ; i ++)
    {
        if(point_2d_2[i].empty())
        {
            ranges_degree[2*i] = INFINITY;
            ranges_degree[2*i -1] = INFINITY;
            continue;
        }

        double t_range = sqrt((points_2d_3[i].x * points_2d_3[i].x)+ (points_2d_3[i].y * points_2d_3[i].y));
        ranges_degree[2*i]=t_range;
        ranges_degree[2*i -1] = t_range;
    }
    ranges_degree[0] = ranges_degree[1];
    v_laserScan.clear();
    v_laserScan = ranges_degree;
    // for(int i = 0;i < v_laserScan.size() ; i ++)
    // {
    //     std::cout << "v laser scan ["<< i<<"] "<< v_laserScan[i] << std::endl;
    // }
    if(control_msg_buffer.size() > 0)
    {
        robot_pub.publish(control_msg_buffer.back());
    }
    //time2++;
    //plotLidarCluster(points_2d_3,361);
}

void robotScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    //std::cout<< "robotScanCallback in" << std::endl; 
    if(!v_laserScan.empty())
    {
        v_laserScan.clear();
    }
    std::vector<std::vector<carmen_point_t>> point_2d_v(361);
    std::vector<std::vector<double>> ranges_degree_v(361);
    std::vector<double> ranges_degree(361);
    ////std::cout << "laser size : "<<msg->ranges.size() << std::endl;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        // if (i % 2 == 0)
        // {
        //     ranges_degree.push_back(msg->ranges[i]);
        // }
        if (!std::isfinite(msg->ranges[i])) // std::isfinite ：Returns true if the value entered is a valid value
        {
            continue;
        }
        double angle = msg->angle_min + (msg->angle_increment * i);
        double t_x = msg->ranges[i] * cos(angle);
        double t_y = msg->ranges[i] * sin(angle);
        double angle_degree = ((double)180.0) / M_PI * angle +90.0;
        if(angle_degree<0)
        {
            angle_degree +=360.0;
        }
        //angle_degree += 180.0;
        int index = floor(angle_degree);

        ////std::cout << "index : "<< index << std::endl;
        if(index < 0)
        {
            continue;
        }
        carmen_point_t point_temp;
        point_temp.x = t_x;
        point_temp.y = t_y;
        point_temp.theta = angle_degree;
        point_2d_v[index].push_back(point_temp);
        ranges_degree_v[index].push_back(msg->ranges[i]);
    }
    //std::cout << "raw data were set ! " << std::endl;
    
    for (int i = 1; i < 180; i ++ )
    {
        if(point_2d_v[i].empty())
        {
            ranges_degree[2*i] = 2.2;
            ranges_degree[2*i -1] = 2.2;
            continue;
        }
        if(i< 55 || i > 125)
        {
            ranges_degree[2*i] = 2.2;
            ranges_degree[2*i - 1] = 2.2;
            continue;
        }
        int t_size = point_2d_v[i].size();
        double t_x_m = 0,t_y_m = 0,range_m = 0;
        int count = 0;
        for(int j = 0; j < t_size ; j++)
        {
            t_x_m += point_2d_v[i][j].x;
            t_y_m += point_2d_v[i][j].y;
            range_m += ranges_degree_v[i][j];
            count++;
        }
        range_m = range_m / (double)count;
        ranges_degree[2*i] = range_m;
        ranges_degree[2*i - 1] = range_m;
    }
    //ranges_degree.push_back(10.0);
    ranges_degree[0] = ranges_degree[1];
    ranges_degree[359] = ranges_degree[358];
    ranges_degree[360] = ranges_degree[1];
    v_laserScan = ranges_degree;
    // for(int i = 0; i < 361 ; i ++)
    // {
    //     v_laserScan.push_back(ranges_degree[i]);
    // }
    //std::cout << "v_laserScan data were set ! " << std::endl;
    // for(int i = 0;i < v_laserScan.size() ; i ++)
    // {
    //     std::cout << "v laser scan ["<< i<<"] "<< v_laserScan[i] << std::endl;
    // }
    if(!control_msg_buffer.empty())
    {
        robot_pub.publish(control_msg_buffer.back());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_control");
    ros::NodeHandle nh;
    currentVel.linear.x = 0;
    currentVel.angular.z = 0;
    ros::Subscriber sub_robot_vel = nh.subscribe("/robot2/cmd_vel_mux/input/teleop", 1000, robotCmdVelCallback);    
    ros::Subscriber sub_livox_scan = nh.subscribe("/livox/lidar", 1000, callback);
    //ros::Subscriber sub_robot_scan = nh.subscribe("/robot2/scan", 1000, robotScanCallback);
    //ros::Subscriber sub_relative_pose = nh.subscribe("/UAV1HumanRelativePose", 1000, callback);
    //robot_pub = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel_mux/input/teleop", 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose1_sub(nh, pose1_topic, 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose2_sub(nh, pose2_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> controlSyncPolicy;
    message_filters::Synchronizer<controlSyncPolicy> sync(controlSyncPolicy(30), pose1_sub, pose2_sub);
    sync.registerCallback(boost::bind(&callback_relative, _1, _2));
    robot_pub = nh.advertise<geometry_msgs::Twist>("robot2/cmd_vel_mux/input/teleop", 1);
    ros::spin();
    return 0;
}
