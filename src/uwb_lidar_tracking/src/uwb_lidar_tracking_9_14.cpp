/*重写版 优化过的版本 从文件读取参数*/
/*没有粒子滤波*/
/*针对静态障碍物 优化 避免错误识别*/
/*输出物体状态(概率)*/
/*uwb ranging 版本*/
/*ver:2024.7.24*/
/*ver:2024.8.20 加入了svm的svdd定位, 加入KD Tau, readpara加入了几个选择*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <ctime>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <uwb_lidar_tracking/Target.h>
#include <uwb_lidar_tracking/Targets.h>
#include <uwb_lidar_tracking/UltraWBAoaNode.h>
#include <uwb_lidar_tracking/UltraWBAoa.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/LinktrackAoaNodeframe0.h>

#include "libsvm.h"
#include "libsvm.cpp"

const char* svm_model_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/src/uwb_lidar_tracking/include/right_feature.txt.model");
svm_model *model = svm_load_model(svm_model_name);

// configuration
ros::Publisher targets_pub;
ros::Publisher cluster_pub;
ros::Publisher object_pub;
ros::Publisher matched_object_pub;
ros::Publisher aoa_trajectory_pub;


std::string lidar_scan_topic = "/robot1/scan" ;//"/fake_scan"; //"/robot1/scan";
std::string uwb_topic = "/robot1/nlink_linktrack_nodeframe2";  //"/robot1/nlink_linktrack_nodeframe2" "/aoa_measurement"

const std::string configure_file_name("/home/gglin/WORKSPACE/uwb_lidar_following_gglin/src/uwb_lidar_tracking/config/localization_configure.txt");

// global parameters
int downsample_flag = 1;

double dbscan_eps;
int dbscan_min;

int trajectory_length;
double nn_threshold;
double lost_time_threshold;      // in sec
int trajectory_length_threshold; // in frame
double identical_distance_threshold;
double similarity_threshold;
int next_object_id = 1;
int localization_mode = 1; // 1 uwb + lidar , 2 lidar only (SVDD)
int similarity_measure = 1; // 1 gaussian, 2 KD tau, 3 cosine

// scan scope
double range_limit;

struct Point
{
    double x;
    double y;
    double r; // range
    double a; // angle
    bool status; // is this point valid or not

    ros::Time measure_time;
    int cluster_id;

    Point()
    {
        x = 0;
        y = 0;
        r = 0;
        a = 0;
        status = false;

        cluster_id = -1;
    }
    Point(double x, double y) : x(x), y(y), status(false) {}
    Point(double x, double y, int id) : x(x), y(y), cluster_id(id), status(false) {}

};

class Cluster
{
public:
    int id;
    bool status; // means: is used in association or not
    std::vector<Point> points;
    Point center;
    double width = 0;
    double girth = 0;
    double depth = 0;
    double area = 0;

    Cluster()
    {
        points.clear();
        id = -1;
        status = false;
        width = 0;
        girth = 0;
        depth = 0;
        area = 0;
    }

    Cluster(int id) : id(id), center(0, 0), status(false) {}

    void getCentroid()
    {
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto &point : this->points)
        {
            sum_x += point.x;
            sum_y += point.y;
        }
        center.x = sum_x / points.size();
        center.y = sum_y / points.size();
    }
    void getWidth()
    {
        width = sqrt(pow(points.front().x - points.back().x, 2) + pow(points.front().y - points.back().y, 2));
    }

};

struct Object
{
    int id;
    int matched_uwb_id; // -1 means no matched uwb

    double x;
    double y;
    double width;

    bool status; // false means noise
    bool is_occluded;

    ros::Time current_updated_time;

    int association_count;

    boost::circular_buffer<Point> trajectory;
    double probability_count; // probability of object is the target
    Object()
    {
        id = -1;
        matched_uwb_id = -1;
        x = 0;
        y = 0;
        width = 100;        // initialize widtd with a large value, cuz if the width > threshold, this object is not our target

        status = false;
        association_count = 0;
        trajectory.resize(trajectory_length);
        trajectory.clear();
        probability_count = 0;
    }
    Object(int obj_id)
    {
        id = obj_id;
        matched_uwb_id = -1;
        x = 0;
        y = 0;
        width = 100;        // initialize widtd with a large value, cuz if the width > threshold, this object is not our target

        status = false;
        association_count = 0;
        trajectory.resize(trajectory_length);
        trajectory.clear();
        probability_count = 0;
    }
};

struct UltraWM
{
    int role;
    double range;
    double angle;
    double x;
    double y;

    ros::Time measure_time;

    UltraWM()
    {
        range = 0;
        angle = 0;
        x = 0;
        y = 0;
        role = 0;
    }
};

// Ultra Wide Band node
class UltraWB
{
public:
    int id;
    int matched_object_id;
    bool is_matched_object_occluded;
    UltraWM current_measure;

    std::vector<UltraWM> v_current_measure;         // used for multi aoa measure at this time
    boost::circular_buffer<UltraWM> s_uwb_meas;     // uwb measurement sequence
    boost::circular_buffer<Point> trajectory;       // associated trajectory from the lidar
    boost::circular_buffer<Point> trajectory_raw;   // raw trajectory from the uwb aoa

    UltraWB()
    {
        id = -1;
        matched_object_id = -1;
        is_matched_object_occluded = false;
        s_uwb_meas.resize(trajectory_length);
        s_uwb_meas.clear();
        trajectory.resize(trajectory_length);
        trajectory.clear();
        trajectory_raw.resize(trajectory_length);
        trajectory_raw.clear();
    }
};

// global map
std::map<int, UltraWB> m_uwbs;
std::map<int, Object> m_objects;
std::map<int, std_msgs::ColorRGBA> color_map;

double degreesToRadians(double degrees)
{
    double radians = degrees * (M_PI / 180.0);

    // Normalize to the range [-pi, pi]
    while (radians > M_PI)
        radians -= 2.0 * M_PI;
    while (radians < -M_PI)
        radians += 2.0 * M_PI;

    return radians;
}

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
        if (std::getline(iss, key, ':') && std::getline(iss, value_str))
        {
            std::istringstream value_stream(value_str);

            if (key == "dbscan_eps")
            {
                value_stream >> dbscan_eps;
                std::cout << "dbscan_eps: \t\t\t" << dbscan_eps << std::endl;
            }
            else if (key == "dbscan_min")
            {
                value_stream >> dbscan_min;
                std::cout << "dbscan_min: \t\t\t" << dbscan_min << std::endl;
            }
            else if (key == "trajectory_length")
            {
                value_stream >> trajectory_length;
                std::cout << "trajectory_length: \t\t" << trajectory_length << std::endl;
            }
            else if (key == "nn_threshold")
            {
                value_stream >> nn_threshold;
                std::cout << "nn_threshold: \t\t\t" << nn_threshold << std::endl;
            }
            else if (key == "lost_time_threshold")
            {
                value_stream >> lost_time_threshold;
                std::cout << "lost_time_threshold: \t\t" << lost_time_threshold << std::endl;
            }
            else if (key == "trajectory_length_threshold")
            {
                value_stream >> trajectory_length_threshold;
                std::cout << "trajectory_length_threshold: \t" << trajectory_length_threshold << std::endl;
            }
            else if (key == "identical_distance_threshold")
            {
                value_stream >> identical_distance_threshold;
                std::cout << "identical_distance_threshold: \t" << identical_distance_threshold << std::endl;
            }
            else if (key == "similarity_threshold")
            {
                value_stream >> similarity_threshold;
                std::cout << "similarity_threshold: \t\t" << similarity_threshold << std::endl;
            }
            else if (key == "range_limitation")
            {
                value_stream >> range_limit;
                std::cout << "range_limitation: \t\t" << range_limit << std::endl;
            }
            else if (key == "localization_mode")
            {
                value_stream >> localization_mode;
                std::cout << "localization_mode(default 1): \t\t" << localization_mode << std::endl;
            }
            else if (key == "similarity_measure")
            {
                value_stream >> similarity_measure;
                std::cout << "similarity_measure(default 1): \t\t" << similarity_measure << std::endl;
            }
        }
    }
    file.close();
}

void initializeColorMap()
{
    std_msgs::ColorRGBA color;

    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0; // Yellow
    color_map[0] = color;

    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 1.0; // Red
    color_map[1] = color;

    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0; // Blue
    color_map[2] = color;

    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.5;
    color.a = 1.0; // Grey
    color_map[3] = color;

    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0; // Green
    color_map[4] = color;

    color.r = 0.0;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 1.0; // Cyan
    color_map[5] = color;

    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0; // Magenta
    color_map[6] = color;

    color.r = 1.0;
    color.g = 0.5;
    color.b = 0.0;
    color.a = 1.0; // Orange
    color_map[7] = color;

    color.r = 0.5;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0; // Purple
    color_map[8] = color;

    color.r = 0.5;
    color.g = 0.5;
    color.b = 0.0;
    color.a = 1.0; // Olive
    color_map[9] = color;
}

std_msgs::ColorRGBA getColor(int id)
{
    int color_id = id % 10; // Use modulus to get a color ID in range [0, 9]
    if (color_map.find(color_id) != color_map.end())
    {
        return color_map[color_id];
    }
    else
    {
        // Default color if ID not found in the map (should not happen if color_map is properly initialized)
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0; // Purple
        return color;  // Black
    }
}

double euclideanDistance(const Point &a, const Point &b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

std_msgs::ColorRGBA generateColor(int cluster_id)
{
    std_msgs::ColorRGBA color;
    std::srand(cluster_id); // Seed with cluster_id for reproducibility
    color.r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    color.g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    color.b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    color.a = 1.0;
    return color;
}

void plotClusters(std::vector<Point> &clustered_points)
{
    visualization_msgs::Marker points_marker;
    // make the frame_id identical with the robot laser frame
    points_marker.header.frame_id = "/follow_frame";
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "clusters";
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::POINTS;

    points_marker.scale.x = 0.2;
    points_marker.scale.y = 0.2;

    for (const auto &point : clustered_points)
    {
        std_msgs::ColorRGBA color;
        if (point.cluster_id > 0)
        {
            color = getColor(point.cluster_id);
        }
        else
        {
            color.r = color.g = color.b = 0.5; // Noise or unclassified
            color.a = 1.0;
        }

        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;

        points_marker.points.push_back(p);
        points_marker.colors.push_back(color);
    }

    cluster_pub.publish(points_marker);
}

void plotTrajectories(const std::map<int, Object> &updated_objects)
{
    visualization_msgs::Marker lines_marker;
    lines_marker.header.frame_id = "/follow_frame";
    lines_marker.header.stamp = ros::Time::now();
    lines_marker.ns = "trajectories";
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.pose.orientation.w = 1.0;
    lines_marker.id = 1;
    lines_marker.type = visualization_msgs::Marker::POINTS;

    lines_marker.scale.x = 0.1; // Line width
    lines_marker.scale.y = 0.1;

    for (const auto &t_object : updated_objects)
    {
        if (!t_object.second.status)
            continue;

        std_msgs::ColorRGBA color = getColor(t_object.second.id);

        for (const auto &point : t_object.second.trajectory)
        {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;

            if (point.status)
            {
                lines_marker.points.push_back(p);
                lines_marker.colors.push_back(color);
            }
        }
    }

    object_pub.publish(lines_marker);
}

void plotTarget(const Object &target)
{ 
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/follow_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target";
    marker.id = target.id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target.trajectory.back().x;
    marker.pose.position.y = target.trajectory.back().y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color = getColor(1);

    matched_object_pub.publish(marker);
}

void plotTarget(const Point &target)
{ 
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/follow_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target.x;
    marker.pose.position.y = target.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color = getColor(1);

    matched_object_pub.publish(marker);
}

void plotAoaTrajectory(const UltraWB &uwb_node)
{
    visualization_msgs::Marker raw_trajectory_marker;
    raw_trajectory_marker.header.frame_id = "/follow_frame";
    raw_trajectory_marker.header.stamp = ros::Time::now();
    raw_trajectory_marker.ns = "raw_trajectory";
    raw_trajectory_marker.id = uwb_node.id;
    raw_trajectory_marker.type = visualization_msgs::Marker::POINTS;
    raw_trajectory_marker.action = visualization_msgs::Marker::ADD;
    raw_trajectory_marker.pose.orientation.w = 1.0;
    raw_trajectory_marker.scale.x = 0.1;
    raw_trajectory_marker.scale.y = 0.1;

    raw_trajectory_marker.color.r = 1.0;
    raw_trajectory_marker.color.g = 0.0;
    raw_trajectory_marker.color.b = 0.0;
    raw_trajectory_marker.color.a = 1.0;

    for (const auto &point : uwb_node.trajectory_raw)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        raw_trajectory_marker.points.push_back(p);
    }

    aoa_trajectory_pub.publish(raw_trajectory_marker);
}

void getClusterFeatures(Cluster &input_cluster)
{
    input_cluster.width = sqrt(pow(input_cluster.points.front().x - input_cluster.points.back().x, 2) + pow(input_cluster.points.front().y - input_cluster.points.back().y, 2));

    input_cluster.girth = 0;

    double a, b, c, s, max_d = 0;
    c = input_cluster.width;
    for (std::vector<Point>::iterator it_point = input_cluster.points.begin() + 1; it_point != input_cluster.points.end(); ++it_point)
    {
        input_cluster.girth += sqrt(pow((it_point->x - (it_point - 1)->x), 2) + pow((it_point->y - (it_point - 1)->y), 2));
        a = sqrt(pow((input_cluster.points.front().x - it_point->x), 2) + pow((input_cluster.points.front().y - it_point->y), 2));
        b = sqrt(pow((input_cluster.points.back().x - it_point->x), 2) + pow((input_cluster.points.back().y - it_point->y), 2));
        s = 0.5 * (a + b + c);
        double d = (2 * sqrt(s * (s - a) * (s - b) * (s - c))) / c;
        if (d > max_d)
            max_d = d;
    }

    input_cluster.depth = max_d;
    input_cluster.area = input_cluster.depth * input_cluster.width;
}

std::vector<Point> laserScanProcess(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<Point> t_points;
    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
        if (!std::isfinite(scan_msg->ranges[i])) // std::isfinite ：Returns true if the value entered is a valid value
        {
            continue;
        }
        double angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        double range = scan_msg->ranges[i];
        if (range < range_limit && range > 0.2)
        {
            Point t_point(range * cos(angle), range * sin(angle));
            t_point.measure_time = scan_msg->header.stamp;
            t_points.push_back(t_point);
        }
    }
    return t_points;
}

void uwbProcess(const nlink_parser::LinktrackNodeframe2::ConstPtr &uwb_msg)
{
    ros::Time current_time = uwb_msg->header.stamp;

    // Update or create UltraWB nodes
    for (const auto &node : uwb_msg->nodes)
    {

        int node_id = node.id;
        
        if (m_uwbs.find(node_id) != m_uwbs.end())
        {
            // Update existing UltraWB node
            UltraWB &existing_node = m_uwbs[node_id];
            existing_node.current_measure.measure_time = current_time;
            existing_node.current_measure.range = node.dis;
            existing_node.s_uwb_meas.push_back(existing_node.current_measure);
        }
        else
        {
            // Create new UltraWB node
            std::cout << "Create new UltraWB node: " << node_id << std::endl;
            UltraWB new_node;
            new_node.id = node_id;
            new_node.current_measure.measure_time = current_time;
            new_node.current_measure.range = node.dis;
            new_node.s_uwb_meas.push_back(new_node.current_measure);
            m_uwbs[node_id] = new_node;
        }
    }
    
    for (auto it = m_uwbs.begin(); it != m_uwbs.end();)
    {
        if (fabs((current_time - it->second.current_measure.measure_time).toSec()) > lost_time_threshold)
        {
            it = m_uwbs.erase(it);
        }
        else
        {
            ++it;
        }
    }
}



// dbscan algorithm
const inline int regionQuery(std::vector<Point> &input, int p, std::vector<int> &output, double eps)
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

bool expandCluster(std::vector<Point> &input, int p, std::vector<int> &output, int cluster, double eps, int min)
{
    std::vector<int> seeds;

    if (regionQuery(input, p, seeds, eps) < min)
    {
        output[p] = -1;
        return false;
    }
    else
    {
        for (int i = 0; i < (int)seeds.size(); i++)
        {
            output[seeds[i]] = cluster;
        }

        seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

        while ((int)seeds.size() > 0)
        {

            int cp = seeds.front();
            std::vector<int> result;

            if (regionQuery(input, cp, result, eps) >= min)
            {
                for (int i = 0; i < (int)result.size(); i++)
                {

                    int rp = result[i];

                    if (output[rp] < 1)
                    {
                        if (!output[rp])
                        {
                            seeds.push_back(rp);
                        }
                        output[rp] = cluster;
                    }
                }
            }

            seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
        }
    }

    return true;
}

int dbscan(std::vector<Point> &input, double eps, int min)
{
    int size = input.size();
    int cluster = 1;

    std::vector<int> state(size);

    for (int i = 0; i < size; i++)
    {
        if (!state[i])
        {

            if (expandCluster(input, i, state, cluster, eps, min))
            {
                cluster++;
            }
        }
    }
    for (int i = 0; i < size; i++)
    {
        input[i].cluster_id = state[i];
    }

    return cluster - 1;
}

std::map<int, Cluster> getClusters(std::vector<Point> &current_points)
{
    dbscan(current_points, dbscan_eps, dbscan_min);
    
    // Create clusters from points
    std::map<int, Cluster> m_t_clusters;
    for (const auto &t_point : current_points)
    {
        if (t_point.cluster_id > 0)
        {
            if (m_t_clusters.find(t_point.cluster_id) == m_t_clusters.end())
            {
                //create new cluster
                m_t_clusters[t_point.cluster_id] = Cluster(t_point.cluster_id);
            }
            m_t_clusters[t_point.cluster_id].points.push_back(t_point);
        }
    }

    plotClusters(current_points);
    return m_t_clusters;
}

bool rangingValidation(Point t_point)
{
    double range_p = sqrt(pow(t_point.x, 2) + pow(t_point.y, 2));
    double min_diff = std::numeric_limits<double>::max();
    for (auto &t_uwb : m_uwbs)
    {
        double range_diff = fabs(t_uwb.second.current_measure.range - range_p);
        if (range_diff < min_diff)
        {
            min_diff = range_diff;
        }
    }
    if (min_diff < identical_distance_threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// nearest neighbor, data associating, clusters -> objects
void associateClusters(std::map<int, Cluster> &current_clusters, const ros::Time &current_time)
{
    for (auto &t_cluster : current_clusters)
    {        
        double min_distance = std::numeric_limits<double>::max();
        int best_match_id = -1;

        t_cluster.second.getCentroid();
        getClusterFeatures(t_cluster.second);
        for (auto &t_object : m_objects)
        {
            double distance = euclideanDistance(t_object.second.trajectory.back(), t_cluster.second.center);
            if (distance < min_distance)
            {
                min_distance = distance;
                best_match_id = t_object.second.id;
            }
        }

        if (min_distance < nn_threshold && best_match_id > 0)
        {
            std::cout << "Object " << best_match_id << " is associated with cluster " << t_cluster.first << std::endl;
            Object &best_match = m_objects[best_match_id];
            t_cluster.second.center.status = true;      //center . point . status is used for similarity calculation
            best_match.trajectory.push_back(t_cluster.second.center);
            best_match.association_count++;
            best_match.width = t_cluster.second.width;
            best_match.is_occluded = false;
            best_match.status = best_match.association_count >= trajectory_length_threshold;
            best_match.current_updated_time = current_time;

            t_cluster.second.status = true;
        }
    }

    // the rest un-used clusters, we first associate it with possible lost object
    for (auto &t_cluster : current_clusters)
    {
        if(!t_cluster.second.status)
        {
            double lost_distance = std::numeric_limits<double>::max();
            int lost_match_id = -1;
            double lost_time = 0;
            for (auto &t_object : m_objects)
            {
                if (t_object.second.current_updated_time == current_time)
                {
                    // we dont process this cluster cuz it was been processed
                    continue;
                }

                double distance = euclideanDistance(t_object.second.trajectory.back(), t_cluster.second.center);
                if (lost_distance > distance)
                {
                    lost_distance = distance;
                    lost_match_id = t_object.second.id;
                    lost_time = fabs(current_time.toSec() - t_object.second.current_updated_time.toSec());
                }
            }

            // re-tracking, assuming that the target moving at 1m/s
            if (lost_distance < std::min(lost_time * 0.8, 2.5) && rangingValidation(t_cluster.second.center) && lost_match_id > 0)
            {
                std::cout << "Object " << lost_match_id << " is re-associated with cluster " << t_cluster.first << std::endl;
                Object &lost_match = m_objects[lost_match_id];
                t_cluster.second.center.status = true;
                lost_match.trajectory.push_back(t_cluster.second.center);
                lost_match.width = t_cluster.second.width;
                lost_match.association_count++;
                lost_match.current_updated_time = current_time;
                lost_match.is_occluded = false;
                lost_match.status = (lost_match.association_count >= trajectory_length_threshold);

                t_cluster.second.status = true;
            }
            else
            {
                std::cout << "Creating New Object! ID:" << next_object_id << std::endl;
                Object new_object(next_object_id);
                next_object_id++;
                t_cluster.second.center.status = true;
                new_object.trajectory.push_back(t_cluster.second.center);
                new_object.current_updated_time = current_time;
                new_object.is_occluded = false;
                new_object.width = t_cluster.second.width;
                new_object.association_count ++;
                m_objects[new_object.id] = new_object;

                t_cluster.second.status = true;
            }
        }
    }

    // Update non-associated objects
    for (auto t_object_it = m_objects.begin(); t_object_it != m_objects.end();)
    {
        //std::cout << "Object ID " << t_object_it->second.id << " associated count " << t_object_it->second.association_count << " pro count " << t_object_it->second.probability_count << std::endl;
        if (current_time != t_object_it->second.current_updated_time)
        {
            if (fabs(current_time.toSec() - t_object_it->second.current_updated_time.toSec()) > lost_time_threshold)
            {
                ROS_INFO("Object %d is lost for more than %2f seconds, deleted!", t_object_it->second.id, lost_time_threshold);
                t_object_it = m_objects.erase(t_object_it);
            }
            else
            {
                std::cout << "Object " << t_object_it->second.id << " is lost." << std::endl;
                Point lost_point = t_object_it->second.trajectory.back();
                lost_point.status = false;
                t_object_it->second.trajectory.push_back(lost_point);
                t_object_it->second.is_occluded = true;
                ++t_object_it;
            }
        }
        else 
        {
            ++t_object_it;
        }
    }

    plotTrajectories(m_objects); 
}

double getGaussian(const Object &input_obj, const UltraWB &input_uwb)
{
    // we should only consider the valid lidar points when calculating similarity
    if (input_obj.trajectory.size() < 5 || input_uwb.s_uwb_meas.size() < 5)
    {
        if(input_obj.association_count < 5)
        {
            return 0.0;
        }
    }

    int valid_size = std::min(input_obj.trajectory.size(), input_uwb.s_uwb_meas.size());

    double distance_sum = 0, distance_avg = 0, distance_count = 0, sigma = 1.0;

    auto it_uwb_s = input_uwb.s_uwb_meas.end();
    auto it_obj_s = input_obj.trajectory.end();

    for (int i = 1; i <= valid_size; ++i)
    {
        double obj_range = std::sqrt(std::pow((it_obj_s - i)->x, 2) + std::pow((it_obj_s - i)->y, 2));
        double uwb_range = (it_uwb_s - i)->range;

        // we only consider the valid point
        if ((it_obj_s - i)->status)
        {
            distance_sum += std::fabs(uwb_range - obj_range);
            distance_count += 1;
        }
    }
    distance_avg = distance_sum / distance_count;

    return std::exp(-0.5 * distance_avg / std::pow(sigma, 2));
}

double getKDTau(const Object &input_obj, const UltraWB &input_uwb)
{
    if (input_obj.trajectory.size() < 5 || input_uwb.s_uwb_meas.size() < 5)
    {
        if(input_obj.association_count < 5)
        {
            return 0.0;
        }
    }
    // Intercept the same length of the segment, the length is the smaller of the two input value
    int valid_size = std::min(input_obj.trajectory.size(), input_uwb.s_uwb_meas.size());

    auto it_uwb_s = input_uwb.s_uwb_meas.end();
    auto it_obj_s = input_obj.trajectory.end();

    double count = 0, identical_count = 0;

    for (int i = 1; i <= valid_size; ++i)
    {
        double obj_range = std::sqrt(std::pow((it_obj_s - i)->x, 2) + std::pow((it_obj_s - i)->y, 2));
        double uwb_range = (it_uwb_s - i)->range;

        // we only consider the valid point
        if ((it_obj_s - i)->status)
        {
            count = count + 1.0;
            if(fabs(obj_range - uwb_range) < 0.5)
            {
                identical_count = identical_count + 1.0;
            }
        }
    }
    if(identical_count < 0.1)
    {
        return 0.0;
    }
    else
    {
        return identical_count / count;
    }
}

double getCosine(const Object &input_obj, const UltraWB &input_uwb)
{
    if (input_obj.trajectory.size() < 5 || input_uwb.s_uwb_meas.size() < 5)
    {
        if(input_obj.association_count < 5)
        {
            return 0.0;
        }
    }
    double uwb_sum = 0, obj_sum = 0, count_sum = 0;
    double cross_sum = 0, cross_count = 0;
    int valid_size = std::min(input_obj.trajectory.size(), input_uwb.s_uwb_meas.size());

    auto it_uwb_s = input_uwb.s_uwb_meas.end();
    auto it_obj_s = input_obj.trajectory.end();
    for (int i = 1; i <= valid_size; ++i)
    {
        if ((it_obj_s - i)->status)
        {
            double point_R = std::sqrt(std::pow((it_obj_s - i)->x, 2) + std::pow((it_obj_s - i)->y, 2));
            uwb_sum += (it_uwb_s - i)->range * (it_uwb_s - i)->range;
            obj_sum += point_R * point_R;
            count_sum++;
        }
    }; 
    uwb_sum = std::max(uwb_sum, 0.1);
    obj_sum = std::max(obj_sum, 0.1);

    for (int i = 1; i <= valid_size; ++i)
    {
        if ((it_obj_s - i)->status)
        {
            double point_R = std::sqrt(std::pow((it_obj_s - i)->x, 2) + std::pow((it_obj_s - i)->y, 2));
            cross_sum += (it_uwb_s - i)->range * point_R;
            cross_count ++;
        }
    }; 
    double cosine_similarity = cross_sum / (sqrt(uwb_sum) * sqrt(obj_sum));

    return cosine_similarity;
}

void sequenceLocalization(ros::Time current_time)
{
    uwb_lidar_tracking::Targets t_targets;
    t_targets.targets.clear();
    t_targets.header.stamp = current_time;
    t_targets.header.frame_id = "test";

    // record and publish the most match object for each uwb node
    for (auto &uwb_node : m_uwbs)
    {
        uwb_lidar_tracking::Target t_target;
        if (uwb_node.second.matched_object_id > 0)
        {
            if (m_objects.find(uwb_node.second.matched_object_id) == m_objects.end())
            {
                t_target.is_occluded = true; // this object was deleted
            }
            else if (m_objects[uwb_node.second.matched_object_id].is_occluded)
            {
                t_target.is_occluded = true;
            }
            else
            {
                t_target.is_occluded = false;
            }
        }
        else
        {
            t_target.is_occluded = true;
        }

        double max_similarity = -std::numeric_limits<double>::infinity();
        double max_similarity_KD = -std::numeric_limits<double>::infinity();

        int matched_obj_index = -1;

        for (auto &t_object : m_objects)
        {
            if (t_object.second.status && !t_object.second.is_occluded && t_object.second.width < 3) //在这里利用特征筛选一次 会提高匹配效果
            {
                double temp_similarity = 0;
                if(similarity_measure == 1)
                {
                    temp_similarity = getGaussian(t_object.second, uwb_node.second);
                    std::cout << "Gaussian: " << temp_similarity << std::endl;
                }
                else if(similarity_measure == 2)
                {
                    temp_similarity = getKDTau(t_object.second, uwb_node.second);
                    std::cout << "KD: " << temp_similarity << std::endl;
                }
                else if(similarity_measure == 3)
                {
                    temp_similarity = getCosine(t_object.second, uwb_node.second);
                    std::cout << "Cosine: " << temp_similarity << std::endl;
                }
                if (temp_similarity > max_similarity)
                {
                    max_similarity = temp_similarity;
                    matched_obj_index = t_object.second.id;
                }
            }
        }

        if (max_similarity > similarity_threshold && matched_obj_index > 0) //
        {
            m_objects[matched_obj_index].probability_count = m_objects[matched_obj_index].probability_count + 1.0;
            double proba_count = m_objects[matched_obj_index].probability_count / (m_objects[matched_obj_index].association_count - trajectory_length_threshold + 1.0);
            t_target.probability = std::min(proba_count, 1.0);
            // prevent from localization result jumping

            if (t_target.probability > 0.3) //在这里利用特征筛选一次 效果不明显 && euclideanDistance(m_objects[matched_obj_index].trajectory.back(), uwb_node.second.trajectory.back()) < nn_threshold
            {
                t_target.id = matched_obj_index;
                t_target.uwb_id = uwb_node.second.id;
                t_target.status = true;
                t_target.x = m_objects[matched_obj_index].trajectory.back().x;
                t_target.y = m_objects[matched_obj_index].trajectory.back().y;
                t_target.is_occluded = false;
                uwb_node.second.matched_object_id = matched_obj_index;
                uwb_node.second.trajectory.push_back(m_objects[matched_obj_index].trajectory.back());

                m_objects[matched_obj_index].matched_uwb_id = uwb_node.second.id;
                t_targets.targets.push_back(t_target);
                plotTarget(m_objects[matched_obj_index]);

                // for long-term run, re-set the probability to 1;
                if(m_objects[matched_obj_index].probability_count > trajectory_length)
                {
                    m_objects[matched_obj_index].association_count = 0.25 * trajectory_length + trajectory_length_threshold;
                    m_objects[matched_obj_index].probability_count = 0.25 * trajectory_length;
                }
            }
            else
            {
                std::cout<< "Filtered wrong lidar localization!!" << std::endl;
                t_target.id = matched_obj_index;
                t_target.uwb_id = uwb_node.second.id;
                t_target.status = false;
                t_target.x = uwb_node.second.trajectory.back().x;
                t_target.y = uwb_node.second.trajectory.back().y;
                t_targets.targets.push_back(t_target);
            }
        }
        else // similarity is not ok
        {
            t_target.probability = 0;
            t_target.status = false;
            t_target.id = -1;
            t_target.uwb_id = uwb_node.second.id;
            t_target.x = uwb_node.second.trajectory_raw.back().x;
            t_target.y = uwb_node.second.trajectory_raw.back().y;
            t_targets.targets.push_back(t_target);
            //plotTarget(uwb_node.second.trajectory_raw.back());
        }
    }
    targets_pub.publish(t_targets);
    t_targets.targets.clear();
}

void targetLocalization(std::map<int, Cluster> &clusters)
{
    for (std::map<int, Cluster>::iterator it_clu = clusters.end(); it_clu != clusters.begin(); --it_clu)
    {
        it_clu --;
        if(it_clu == clusters.begin())
        {
            break;
        }
        
        getClusterFeatures(it_clu->second);
        std::cout << "area :" << it_clu->second.area << " depth " << it_clu->second.depth << " girth " << it_clu->second.girth << " width " << it_clu->second.width << std::endl;
        svm_node *svm_predict_input = new svm_node[4 + 1];
        svm_predict_input[0].index = 0;
        svm_predict_input[0].value = it_clu->second.area;
        svm_predict_input[1].index = 1;
        svm_predict_input[1].value = it_clu->second.depth;
        svm_predict_input[2].index = 2;
        svm_predict_input[2].value = it_clu->second.girth;
        svm_predict_input[3].index = 3;
        svm_predict_input[3].value = it_clu->second.width;
        svm_predict_input[4].index = -1;
        double predict_result = svm_predict(model, svm_predict_input);
        if(predict_result > 0)
        {
            uwb_lidar_tracking::Targets t_targets;
            t_targets.targets.clear();
            t_targets.header.stamp = ros::Time::now();
            t_targets.header.frame_id = "follow_frame";
            uwb_lidar_tracking::Target t_target;

            t_target.id = 1;
            t_target.uwb_id = 1;
            t_target.status = false;
            it_clu->second.getCentroid();
            t_target.x = it_clu->second.center.x;
            t_target.y = it_clu->second.center.y;
            t_target.probability = 0;
            t_target.is_occluded = false;
            t_targets.targets.push_back(t_target);

            std::cout << "SVDD Localization! predict_result: " << predict_result << std::endl;
            targets_pub.publish(t_targets);
            break;
        }
    }
}

void Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const nlink_parser::LinktrackNodeframe2::ConstPtr &uwb_msg)
{
    if (downsample_flag % 4 != 0)
    {
        downsample_flag++;
        return;
    }
    else
    {
        ros::Time current_measure_time = scan_msg->header.stamp;
        // get laser point
        std::vector<Point> v_t_points = laserScanProcess(scan_msg);

        // get clusters
        std::map<int, Cluster> m_clusters = getClusters(v_t_points);

        if(localization_mode > 1)
        {
            targetLocalization(m_clusters);
            return;
        }

        // data association
        associateClusters(m_clusters, current_measure_time);

        // get ranging sequences
        uwbProcess(uwb_msg);

        sequenceLocalization(current_measure_time);
        downsample_flag = 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_lidar_tracking");
    ros::NodeHandle nh;

    readParametersFromFile(configure_file_name);
    initializeColorMap();

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, lidar_scan_topic, 1);
    message_filters::Subscriber<nlink_parser::LinktrackNodeframe2> range_sub(nh, uwb_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nlink_parser::LinktrackNodeframe2> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(10), scan_sub, range_sub);

    sync.registerCallback(boost::bind(&Callback, _1, _2));

    cluster_pub = nh.advertise<visualization_msgs::Marker>("/cluster_ploter", 1);
    object_pub = nh.advertise<visualization_msgs::Marker>("/object_ploter", 1);
    matched_object_pub = nh.advertise<visualization_msgs::Marker>("/matched_object_ploter", 1);
    aoa_trajectory_pub = nh.advertise<visualization_msgs::Marker>("/aoa_trajectory_ploter", 1);

    targets_pub = nh.advertise<uwb_lidar_tracking::Targets>("/targets", 1);

    while (nh.ok())
    {
        ros::spinOnce();
    }
    return 0;
}