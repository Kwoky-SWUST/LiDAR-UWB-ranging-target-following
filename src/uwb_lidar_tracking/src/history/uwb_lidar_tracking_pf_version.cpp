/*重写版 优化过的版本 从文件读取参数*/
/*带粒子滤波*/

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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <pthread.h>
#include <iostream>
#include <fstream>

#include "Target.h"
#include "Targets.h"

#include "nlink_parser/LinktrackNodeframe2.h"

// configuration
ros::Publisher targets_pub;
ros::Publisher cluster_pub;
ros::Publisher object_pub;
ros::Publisher matched_object_pub;
std::random_device rd;
std::mt19937 gen(rd());

std::string lidar_scan_topic = "/fake_scan"; //"/robot1/scan";
std::string uwb_topic = "/nlink_linktrack_nodeframe2";  //"/robot1/nlink_linktrack_nodeframe2"

const std::string configure_file_name("/home/gglin/WORKSPACE/uwb_lidar_following/src/uwb_lidar_tracking/config/localization_configure.txt");

// global parameters
double dbscan_eps;
int dbscan_min;

int trajectory_length;
double nn_threshold;
double lost_time_threshold;             // in sec
int trajectory_length_threshold;        // in frame
double identical_distance_threshold;
double similarity_threshold;
int next_object_id = 1;

// scan scope
double range_limit;


// switch
bool flag_amcl_record = true;
bool flag_save_r_feature = false;
bool flag_save_output = true;

bool flag_object_debug = false;
bool flag_uwb_debug = false;
bool flag_obj_similarity_debug = false;

bool flag_plot_clusters = true;
bool flag_plot_objects = true;
bool flag_plot_uwb_history = false;

bool flag_is_svm_model_created = true;
bool flag_particle_spreading = false;

struct Point
{
    double x;
    double y;

    ros::Time measure_time;
    int cluster_id;

    Point()
    {
        x = 0;
        y = 0;
        cluster_id = -1;
    }
    Point(double x, double y) : x(x), y(y) {}
    Point(double x, double y, int id) : x(x), y(y), cluster_id(id) {}
};

struct Particle
{
    double x, y;
    double weight;
    Particle(double x = 0, double y = 0, double weight = 1.0) : x(x), y(y), weight(weight) {}
};

class Cluster
{
public:
    int id;
    bool status;
    std::vector<Point> points;
    Point center;

    double width; // features

    Cluster()
    {
        points.clear();
        id = -1;
        status = false;

        width = 0;
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
        if (this->points.empty())
        {
            width = 0;
            return;
        }
        width = std::sqrt(std::pow(points.front().x - points.back().x, 2) + std::pow(points.front().y - points.back().y, 2));
    }
};

struct Object
{
    int id;
    int matched_uwb_id;
    double x, y;
    bool status;
    ros::Time current_updated_time;
    int association_count;
    int non_association_count;
    std::vector<Particle> particles;
    boost::circular_buffer<Point> trajectory;

    Object()
    {
        id = -1;
        matched_uwb_id = -1;
        x = 0;
        y = 0;

        status = false;
        association_count = 0;
        trajectory.resize(trajectory_length);
        trajectory.clear();
    }

    void addParticle(const Particle &particle)
    {
        particles.push_back(particle);
    }

    void clearParticles()
    {
        particles.clear();
    }

    void initializeParticles(int num_particles, double x_center, double y_center, double spread)
    {
        std::uniform_real_distribution<> dis_x(x_center - spread, x_center + spread);
        std::uniform_real_distribution<> dis_y(y_center - spread, x_center + spread);
        particles.clear();
        for (int i = 0; i < num_particles; ++i)
        {
            Particle p(dis_x(gen), dis_y(gen), 1.0 / num_particles);
            particles.push_back(p);
        }
    }

    void predictParticles(double delta_x, double delta_y, double delta_theta)
    {
        std::normal_distribution<> dis_x(0, 0.1);
        std::normal_distribution<> dis_y(0, 0.1);
        std::normal_distribution<> dis_theta(0, 0.1);

        for (auto &particle : particles)
        {
            double new_x = particle.x + delta_x + dis_x(gen);
            double new_y = particle.y + delta_y + dis_y(gen);
            double new_theta = delta_theta + dis_theta(gen);

            particle.x = new_x;
            particle.y = new_y;
        }
    }

    double calculateWeight(const Particle &p, const std::vector<double> &uwb_ranges, const std::vector<std::pair<double, double>> &uwb_positions, double sigma)
    {
        double weight = 1.0;
        for (size_t i = 0; i < uwb_ranges.size(); ++i)
        {
            double expected_range = std::sqrt(std::pow(p.x - uwb_positions[i].first, 2) + std::pow(p.y - uwb_positions[i].second, 2));
            weight *= std::exp(-0.5 * std::pow((expected_range - uwb_ranges[i]) / sigma, 2)) / (sigma * std::sqrt(2.0 * M_PI));
        }
        return weight;
    }

    void updateParticleWeights(const std::vector<double> &uwb_ranges, const std::vector<std::pair<double, double>> &uwb_positions, double sigma)
    {
        for (auto &particle : particles)
        {
            particle.weight = calculateWeight(particle, uwb_ranges, uwb_positions, sigma);
        }
    }

    void resampleParticles()
    {
        std::vector<Particle> new_particles;
        double max_weight = (*std::max_element(particles.begin(), particles.end(), [](const Particle &a, const Particle &b)
                                               { return a.weight < b.weight; }))
                                .weight;

        std::uniform_real_distribution<> dist(0.0, max_weight);
        double beta = 0.0;
        int index = std::uniform_int_distribution<>(0, particles.size() - 1)(gen);

        for (int i = 0; i < particles.size(); ++i)
        {
            beta += dist(gen) * 2.0;
            while (beta > particles[index].weight)
            {
                beta -= particles[index].weight;
                index = (index + 1) % particles.size();
            }
            new_particles.push_back(particles[index]);
        }
        particles = new_particles;
    }

    Particle getEstimatedPosition() const
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_weight = 0.0;

        for (const auto &particle : particles)
        {
            sum_x += particle.x * particle.weight;
            sum_y += particle.y * particle.weight;
            sum_weight += particle.weight;
        }

        return Particle(sum_x / sum_weight, sum_y / sum_weight, 1.0);
    }
};

struct UltraWM
{
    double range;
    double rx;
    double fp;
    ros::Time measure_time;

    UltraWM()
    {
        range = 0;
        rx = 0;
        fp = 0;
    }
};

// Ultra Wide Band node
class UltraWB
{
public:
    int id;
    int matched_object_id;
    UltraWM current_measure;

    boost::circular_buffer<UltraWM> s_uwb_meas; // uwb measurement sequence

    UltraWB()
    {
        s_uwb_meas.resize(trajectory_length);
        s_uwb_meas.clear();
    }
};

// global map
std::map<int, UltraWB> m_uwbs;
std::map<int, Object> m_objects;
std::map<int, std_msgs::ColorRGBA> color_map;

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
            }
            else if (key == "dbscan_min")
            {
                value_stream >> dbscan_min;
            }
            else if (key == "trajectory_length")
            {
                value_stream >> trajectory_length;
            }
            else if (key == "nn_threshold")
            {
                value_stream >> nn_threshold;
            }
            else if (key == "lost_time_threshold")
            {
                value_stream >> lost_time_threshold;
            }
            else if (key == "trajectory_length_threshold")
            {
                value_stream >> trajectory_length_threshold;
            }
            else if (key == "identical_distance_threshold")
            {
                value_stream >> identical_distance_threshold;
            }
            else if (key == "similarity_threshold")
            {
                value_stream >> similarity_threshold;
            }
            else if (key == "range_limitation")
            {
                value_stream >> range_limit;
            }
        }
    }
    file.close();
    std::cout << "dbscan_eps: \t\t\t" << dbscan_eps << std::endl;
    std::cout << "dbscan_min: \t\t\t" << dbscan_min << std::endl;
    std::cout << "trajectory_length: \t\t" << trajectory_length << std::endl;
    std::cout << "nn_threshold: \t\t\t" << nn_threshold << std::endl;
    std::cout << "lost_time_threshold: \t\t" << lost_time_threshold << std::endl;
    std::cout << "trajectory_length_threshold: \t" << trajectory_length_threshold << std::endl;
    std::cout << "identical_distance_threshold: \t" << identical_distance_threshold << std::endl;
    std::cout << "similarity_threshold: \t\t" << similarity_threshold << std::endl;
    std::cout << "range_limitation: \t\t" << range_limit << std::endl;
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

void publishParticles(const ros::Publisher &pub, const std::vector<Particle> &particles)
{
    geometry_msgs::PoseArray pose_array;
    for (const auto &p : particles)
    {
        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose_array.poses.push_back(pose);
    }
    pub.publish(pose_array);
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
    points_marker.header.frame_id = "/follow";
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
    lines_marker.header.frame_id = "/follow";
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

            lines_marker.points.push_back(p);
            lines_marker.colors.push_back(color);
        }
    }

    object_pub.publish(lines_marker);
}

void plotTarget(const Object &target)
{ 
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/follow";
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
        if (range < 100)
        {
            Point t_point(range * cos(angle), range * sin(angle));
            t_point.measure_time = scan_msg->header.stamp;
            t_points.push_back(t_point);
        }
    }
    return t_points;
}

void uwbRangingProcess(const nlink_parser::LinktrackNodeframe2::ConstPtr &range_msg)
{
    ros::Time current_time = range_msg->header.stamp;

    // Update or create UltraWB nodes
    for (const auto &node : range_msg->nodes)
    {
        int node_id = node.id;
        if (m_uwbs.find(node_id) != m_uwbs.end())
        {
            // Update existing UltraWB node
            UltraWB &existing_node = m_uwbs[node_id];
            existing_node.current_measure.measure_time = current_time;
            existing_node.current_measure.range = node.dis;
            existing_node.current_measure.fp = node.fp_rssi;
            existing_node.current_measure.rx = node.rx_rssi;
            existing_node.s_uwb_meas.push_back(existing_node.current_measure);
        }
        else
        {
            // Create new UltraWB node
            UltraWB new_node;
            new_node.id = node_id;
            new_node.current_measure.measure_time = current_time;
            new_node.current_measure.range = node.dis;
            new_node.current_measure.fp = node.fp_rssi;
            new_node.current_measure.rx = node.rx_rssi;
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
    double range_p = sqrt(pow(t_point.x,2)+ pow(t_point.y,2));
    double min_diff = std::numeric_limits<double>::max();
    for(auto &t_uwb : m_uwbs)
    {
        double range_diff = fabs(t_uwb.second.current_measure.range - range_p);
        if(range_diff < min_diff)
        {
            min_diff = range_diff;
        }
    }
    if(min_diff < identical_distance_threshold)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void associateClusters(std::map<int, Cluster> &current_clusters, const ros::Time &current_time)
{
    for (auto &t_cluster : current_clusters)
    {        
        double min_distance = std::numeric_limits<double>::max();
        int best_match_id = -1;

        t_cluster.second.getCentroid();
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
            Object &best_match = m_objects[best_match_id];
            best_match.trajectory.push_back(t_cluster.second.center);
            best_match.association_count++;
            best_match.non_association_count = 0;
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
                if (t_object.second.non_association_count < 1)
                {
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
            // re-tracking
            // target moving at 1m/s
            if (lost_distance > std::min(lost_time * 0.8, 2.0) && rangingValidation(t_cluster.second.center) && lost_match_id > 0)
            {
                ROS_INFO("Retracking Successful!");
                Object &lost_match = m_objects[lost_match_id];
                lost_match.trajectory.push_back(t_cluster.second.center);
                lost_match.association_count++;
                lost_match.non_association_count = 0;
                lost_match.status = lost_match.association_count >= trajectory_length_threshold;
                lost_match.current_updated_time = current_time;

                t_cluster.second.status = true;
            }
            else
            {
                Object new_object;
                new_object.id = (next_object_id++);
                new_object.trajectory.push_back(t_cluster.second.center);
                new_object.current_updated_time = current_time;
                m_objects[new_object.id] = new_object;

                t_cluster.second.status = true;
            }
        }
    }

    // Update non-associated objects
    for (auto t_object_it = m_objects.begin(); t_object_it != m_objects.end();)
    {
        if(current_time != t_object_it->second.current_updated_time)
        {
            if (fabs(current_time.toSec() - t_object_it->second.current_updated_time.toSec()) > lost_time_threshold)
            {
                t_object_it = m_objects.erase(t_object_it);
            }
            else
            {
                t_object_it->second.status = false;
                t_object_it->second.non_association_count++;
                ++t_object_it;
            }
        }
        else
        {
            ++t_object_it;
        }
    }
    
    plotTrajectories(m_objects); // Plot trajectories
}

double getGaussianSimilarity(const Object &input_obj, const UltraWB &input_uwb)
{
    if (input_obj.trajectory.size() < 5 || input_uwb.s_uwb_meas.size() < 5)
    {
        return 0.0;
    }

    int valid_size = std::min(input_obj.trajectory.size(), input_uwb.s_uwb_meas.size());

    double distance_sum = 0, distance_avg = 0, distance_count = 0, sigma = 1.0;

    auto it_uwb_s = input_uwb.s_uwb_meas.end();
    auto it_obj_s = input_obj.trajectory.end();

    for (int i = 1; i <= valid_size; ++i)
    {
        double obj_range = std::sqrt(std::pow((it_obj_s - i)->x, 2) + std::pow((it_obj_s - i)->y, 2));
        double uwb_range = (it_uwb_s - i)->range;
        distance_sum += std::fabs(uwb_range - obj_range);
        distance_count += 1;
    }
    distance_avg = distance_sum / distance_count;

    return std::exp(-0.5 * distance_avg / std::pow(sigma, 2));
}

void sequenceLocalization(ros::Time current_time)
{
    uwb_lidar_tracking::Targets t_targets;
    t_targets.targets.clear();
    t_targets.header.stamp = current_time;

    // record and publish the most match object for each uwb node
    for (auto &uwb_node : m_uwbs)
    {
        double max_similarity = -std::numeric_limits<double>::infinity();
        double max_g_similarity = -std::numeric_limits<double>::infinity();
        double max_j_similarity = -std::numeric_limits<double>::infinity();
        int matched_obj_index = -1;
        uwb_lidar_tracking::Target t_target;
        for (auto &t_object : m_objects)
        {
            if (t_object.second.status)
            {
                double g_similarity = getGaussianSimilarity(t_object.second, uwb_node.second);
                if (g_similarity > max_similarity)
                {
                    max_similarity = g_similarity;
                    matched_obj_index = t_object.second.id;
                }
            }
        }
        if (max_similarity > similarity_threshold && matched_obj_index > 0)
        {
            t_target.id = matched_obj_index;
            t_target.uwb_id = uwb_node.second.id;
            t_target.status = true;
            t_target.x = m_objects[matched_obj_index].trajectory.back().x;
            t_target.y = m_objects[matched_obj_index].trajectory.back().y;
            uwb_node.second.matched_object_id = matched_obj_index;
            m_objects[matched_obj_index].matched_uwb_id = uwb_node.second.id;

            t_targets.targets.push_back(t_target);
            plotTarget(m_objects[matched_obj_index]);
        }
    }
    targets_pub.publish(t_targets);
}

void Callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const nlink_parser::LinktrackNodeframe2::ConstPtr &range_msg)
{
    ros::Time current_measure_time = scan_msg->header.stamp;
    // get laser point
    std::vector<Point> v_t_points = laserScanProcess(scan_msg);
    
    // get clusters
    std::map<int, Cluster> m_clusters = getClusters(v_t_points);

    // data association
    associateClusters(m_clusters, current_measure_time);

    // get ranging sequences
    uwbRangingProcess(range_msg);
    
    sequenceLocalization(current_measure_time);
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
    matched_object_pub = nh.advertise<visualization_msgs::Marker>("/matched_object_marker", 1);

    targets_pub = nh.advertise<uwb_lidar_tracking::Targets>("/targets", 1);

    while (nh.ok())
    {
        ros::spinOnce();
    }
    return 0;
}