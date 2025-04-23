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
std::vector<double> v_t_laserScan;
vfh::VfhDriver *vfhDriver = new vfh::VfhDriver("/home/comgroup/WORKSPACE/human_following/src/follow_control/config/robot2VFHConfig.txt");
ros::Publisher robot_pub;
//GnuplotInterface *plot_2d_lidar = new GnuplotInterface();
std::vector<geometry_msgs::Twist> control_msg_buffer;

void robotCmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    std::cout<< "222" << std::endl;
    currentVel.linear.x = msg->linear.x;
    currentVel.angular.z = msg->angular.z; 
    time3++;
}

void callback_relative(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &human_pose, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &robot_pose)
{
    std::cout << "111" << std::endl;
    v_laserScan = v_t_laserScan;
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
    control_msg_buffer.push_back(vel_msg);
    std::cout << "publish: goal range=" << vel_msg.linear.x << " angular=" <<vel_msg.angular.z  <<" \n"<<std::endl;
    time1++;
    
}

void callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    v_t_laserScan.clear();
    std::vector<livox_ros_driver::CustomPoint> points_3d = msg->points;
    std::cout << "livox lidar point size = \t" << points_3d.size() << std::endl;
    carmen_point_t points_2d[361];
    carmen_point_t points_2d_3[361];
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
            //std::cout<< "index = " << index << std::endl;
            if(points_2d[index].theta_z > 0)
            {
                points_2d[index].x = points_3d[i].x;
                points_2d[index].y = points_3d[i].y;
                points_2d[index].theta_z = 0.0;
            }
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
    for(int i = 0; i < 361 ; i ++)
    {
        //std::cout<< "angle:\t" << i << "\tx y:\t" << points_2d[i].x<< " " << points_2d[i].y <<  std::endl;
        //std::cout <<"2d_3:\t"<< points_2d_3[i].x<< " " << points_2d_3[i].y <<std::endl;
        double t_range = sqrt((points_2d_3[i].x * points_2d_3[i].x)+ (points_2d_3[i].y * points_2d_3[i].y));
        v_t_laserScan.push_back(t_range);
    }
    if(control_msg_buffer.size() > 0)
    {
        robot_pub.publish(control_msg_buffer.back());
    }
    time2++;
    //plotLidarCluster(points_2d_3,361);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_control");
    ros::NodeHandle nh;
    currentVel.linear.x = 0;
    currentVel.angular.z = 0;
    ros::Subscriber sub_robot_vel = nh.subscribe("/robot2/cmd_vel_mux/input/teleop", 1000, robotCmdVelCallback);    
    ros::Subscriber sub_livox_scan = nh.subscribe("/livox/lidar", 1000, callback);
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
