//在gazebo上 模拟真实场景

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <boost/circular_buffer.hpp>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ctime>

#include <stdlib.h>
#include <string.h>
#include <uwb_lidar_tracking/Target.h>
#include <uwb_lidar_tracking/Targets.h>


class RobotPose
{
public:
  RobotPose() {}
  double x;
  double y;
  double yaw;
  double timestamp;
};

struct LidarPoint
{
  double x;
  double y;
  double angle;
  double range;
  int label; // clustering result
};

struct LidarCluster
{
  std::vector<LidarPoint> points;
  int lable;
  double center_x;
  double center_y;
  void lidarClusterCenter()
  {
    double x_sum = 0, y_sum = 0;
    for (const auto &point : points)
    {
      x_sum += point.x;
      y_sum += point.y;
    }
    if(points.size() > 0)
    {
      this->center_x = x_sum / points.size();
      this->center_y = y_sum / points.size();
    }
    else
    {
      this->center_x = -520;
      this->center_y = -520;
    }
  }
};

std::vector<LidarPoint> v_laser_scan;
std::vector<LidarCluster> v_lidar_clusters;

std::vector<RobotPose> v_target_pose;
std::vector<nav_msgs::Odometry> v_target_odom;
std::vector<RobotPose> v_follower_pose;
std::vector<nav_msgs::Odometry> v_follower_odom;

ros::Publisher pub_target_info;

const inline int regionQuery(std::vector<LidarPoint> &input, int p, std::vector<int> &output, double eps)
{
    for (int i = 0; i < (int)input.size(); i++)
    {
        double dx = input[i].x - input[p].x;
        double dy = input[i].y - input[p].y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance < eps)
        {
            output.push_back(i);
        }
    }
    return output.size();
}

bool expandCluster(std::vector<LidarPoint> &input, int p, std::vector<int> &output, int cluster, double eps, int min)
{
    std::vector<int> seeds; //这个seeds可以理解成 该聚类可能的点集 寻找第一个核心点时 p点本身也会存在与seeds中

    //先寻找第一个核心点
    if (regionQuery(input, p, seeds, eps) < min)
    //未找到核心点
    {
        // this point is noise
        output[p] = -1;
        return false;
    }
    else
    //寻找到第一个核心点
    {
        // set cluster id
        for (int i = 0; i < (int)seeds.size(); i++)
        {
            //把seed中第i个数据对应的序号取出来 使output->state中该序号的点的 聚类序号赋值
            output[seeds[i]] = cluster;
        }

        // delete paint from seeds
        //当前的P点从seeds中删除 把第一个核心点从seeds中删除 用的方法很高级 把p点放在容器最后 然后把最后的元素删除
        seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

        // seed -> empty
        while ((int)seeds.size() > 0)
        //对该聚类下 未计算的其他点 进行遍历操作
        {

            int cp = seeds.front();
            std::vector<int> result;

            if (regionQuery(input, cp, result, eps) >= min)
            //如果seeds中还有核心点
            {
                for (int i = 0; i < (int)result.size(); i++)
                {

                    int rp = result[i];

                    // this paint is noise or unmarked point 如果该点的标记为-1或者0 即为噪声点或者未标记的点
                    if (output[rp] < 1)
                    {

                        // unmarked point    如果该点是未标记的点 则把该点放入seeds中 并把当前聚类标记赋给该点
                        if (!output[rp])
                        {
                            seeds.push_back(rp);
                        }

                        // set cluster id
                        output[rp] = cluster;
                    }
                }
            }

            // delete point from seeds 把当前的cp点从seeds中删除
            seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
        }
    }

    return true;
}

int dbscan(std::vector<LidarPoint> &input, double eps, int min)
{
    int size = input.size();
    int cluster = 1;

    std::vector<int> state(size); //初始化的state内容是size个0

    for (int i = 0; i < size; i++)
    {
        if (!state[i])
        //对每个元素都进行操作
        {

            if (expandCluster(input, i, state, cluster, eps, min))
            // 引用的操作 如果在前面的expand_cluster函数中 对output进行过操作的话 那state也会跟着变化
            {
                cluster++;
            }
        }
    }
    for (int i = 0; i < size; i++)
    {
        input[i].label = state[i];
    }

    return cluster - 1;
}

void followerScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // std::cout << "robot2ScanCallback in " << std::endl;
  v_laser_scan.clear();
  const double LIDAR_ERR = 0.10;
  const double LIDAR_MAX = 10;
  double min_dis = 3;
  double min_ang = 0;
  RobotPose min_point;

  for (int i = 0; i < msg->ranges.size(); i++)
  {
    LidarPoint temp_point;
    double angle = 0, range = 0;
    if (msg->ranges[i] >= LIDAR_ERR && msg->ranges[i] <= LIDAR_MAX) // std::isfinite ：Returns true if the value entered is a valid value
    {
      angle = msg->angle_min + msg->angle_increment * i;
      range = msg->ranges[i];
      temp_point.x = range * cos(angle);
      temp_point.y = range * sin(angle);
      temp_point.angle = angle;
      temp_point.range = range;
      temp_point.label = -1;
      v_laser_scan.push_back(temp_point);
    }
  }
  int cluster_num = dbscan(v_laser_scan, 0.3, 4);

  if(cluster_num < 1)
    return;
  
  for(int i = 1; i < cluster_num +1; i ++)
  {
    LidarCluster temp_cluster;
    for(int j = 0; j < v_laser_scan.size(); j++)
    {
      if(v_laser_scan[j].label == i)
      {
        temp_cluster.points.push_back(v_laser_scan[j]);
      }
    }
    v_lidar_clusters.push_back(temp_cluster);
  }
}

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  double timestamp = msg->header.stamp.toSec();
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose pose;
  pose.timestamp = timestamp;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.yaw = yaw;

  std::cout<< "Target Pose: " << pose.x << ", " << pose.y << ", " << pose.yaw << std::endl;

  nav_msgs::Odometry target_odom = *msg;
  v_target_pose.push_back(pose);
  v_target_odom.push_back(target_odom);
}

void followerPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

  double timestamp = msg->header.stamp.toSec();

  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RobotPose follower_pose;
  follower_pose.timestamp = timestamp;
  follower_pose.x = msg->pose.pose.position.x;
  follower_pose.y = msg->pose.pose.position.y;
  follower_pose.yaw = yaw;

  std::cout<< "Follower Pose: " << follower_pose.x << ", " << follower_pose.y << ", " << follower_pose.yaw << std::endl;

  v_follower_pose.push_back(follower_pose);

  nav_msgs::Odometry follower_odom = *msg;

  if (!v_target_odom.empty())
  {
    // 计算相对位置
    nav_msgs::Odometry target_odom = v_target_odom.back();

    tf::Pose target_tf_pose;
    target_tf_pose.setOrigin(tf::Vector3(target_odom.pose.pose.position.x, target_odom.pose.pose.position.y, target_odom.pose.pose.position.z));
    tf::Quaternion target_tf_q;
    target_tf_q.setW(target_odom.pose.pose.orientation.w);
    target_tf_q.setX(target_odom.pose.pose.orientation.x);
    target_tf_q.setY(target_odom.pose.pose.orientation.y);
    target_tf_q.setZ(target_odom.pose.pose.orientation.z);
    target_tf_pose.setRotation(target_tf_q);

    tf::Pose follower_tf_pose;
    follower_tf_pose.setOrigin(tf::Vector3(follower_odom.pose.pose.position.x, follower_odom.pose.pose.position.y, follower_odom.pose.pose.position.z));
    tf::Quaternion follower_tf_q;
    follower_tf_q.setW(follower_odom.pose.pose.orientation.w);
    follower_tf_q.setX(follower_odom.pose.pose.orientation.x);
    follower_tf_q.setY(follower_odom.pose.pose.orientation.y);
    follower_tf_q.setZ(follower_odom.pose.pose.orientation.z);
    follower_tf_pose.setRotation(follower_tf_q);

    tf::Pose relative_pose;
    relative_pose = follower_tf_pose.inverse() * target_tf_pose;
    tf::Quaternion relative_q = relative_pose.getRotation();

    double relative_roll, relative_pitch, relative_yaw;
    tf::Matrix3x3(relative_q).getRPY(relative_roll, relative_pitch, relative_yaw);
    tf::Vector3 relative_pose_v = relative_pose.getOrigin();

    double relative_x, relative_y;
    relative_x = relative_pose_v.x();
    relative_y = relative_pose_v.y();

    std::cout<< "Target Relative Pose: " << relative_x << ", " << relative_y << std::endl;

    uwb_lidar_tracking::Targets target_list;
    target_list.header.stamp = msg->header.stamp;

    uwb_lidar_tracking::Target t_target;
    t_target.x = relative_x;
    t_target.y = relative_y;
    t_target.status = true;
    t_target.id = 2;
    t_target.uwb_id = 1;

    target_list.targets.push_back(t_target);

    pub_target_info.publish(target_list);
    std::cout << "publish target_list size : " << target_list.targets.size() << std::endl;
    //获取target的 激光聚类
    // if(!v_lidar_clusters.empty())
    // {
    //   for(int i = 0; i < v_lidar_clusters.size(); i++)
    //   {
    //     LidarCluster temp_cluster = v_lidar_clusters[i];
    //     temp_cluster.lidarClusterCenter();
    //     //std::cout << " juli test " << fabs(temp_cluster.center_x - relative_x)+fabs(temp_cluster.center_y - relative_y) <<std::endl;
    //     if(fabs(temp_cluster.center_x - relative_x)+fabs(temp_cluster.center_y - relative_y) < 0.3)
    //     {
    //       for(int j = 0; j < temp_cluster.points.size(); j++)
    //       {
    //         target_info.cluster_angles.push_back(temp_cluster.points[j].angle);
    //         target_info.cluster_ranges.push_back(temp_cluster.points[j].range);
    //       }
    //       break;
    //     }
    //   }
    // }
    //std::cout<< "Target Cluster Size: " << target_info.cluster_angles.size() << ", " << target_info.cluster_ranges.size() << std::endl;
    // if(target_info.cluster_angles.size() > 0 || target_info.cluster_ranges.size()>0)
    // {
    //   pub_target_info.publish(target_info);
    // } 
  }
  v_lidar_clusters.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;

  pub_target_info = nh.advertise<uwb_lidar_tracking::Targets>("/targets", 1);

  ros::Subscriber sub_target_pose = nh.subscribe("/robot1/odom", 1000, targetPoseCallback);
  ros::Subscriber sub_follower_pose = nh.subscribe("/robot2/odom", 1000, followerPoseCallback);

  ros::Subscriber sub_robot2_scan = nh.subscribe("/robot2/scan", 1000, followerScanCallback);

  ros::spin();
  return 0;
}
