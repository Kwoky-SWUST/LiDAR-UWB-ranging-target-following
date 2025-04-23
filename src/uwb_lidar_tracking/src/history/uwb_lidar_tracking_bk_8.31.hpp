#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <vector>
#include <ctime>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen> 
#include <pthread.h>

#include "gnuplot.cpp"
#include "carmen.h"
#include "carmen.cpp"
#include "to_string.h"
#include "pf2_6d.h"

#include "nlink_parser/LinktrackNodeframe2.h"

//configuration
std::string lidar_scan_topic = "/robot1/scan";
std::string uwb_topic = "/nlink_linktrack_nodeframe2";
std::string amcl_file_name("/home/gglin/WORKSPACE/uwb_lidar_tracking/dataset/amcl_record.txt");
std::string uwb_file_name("/home/gglin/WORKSPACE/uwb_lidar_tracking/dataset/uwb_record.txt");
std::string amcl_topic = "/amcl_pose";
bool        flag_amcl_record            = false;

bool        flag_object_debug           = false;
bool        flag_uwb_debug              = false;
bool        flag_obj_similarity_debug   = false;

bool        flag_save_output            = false;

bool        flag_plot_clusters          = true;
bool        flag_plot_objects           = true;
bool        flag_plot_uwb_history       = false;

//type definition
typedef pf2::ParticleFilter6D   ParticleFilter;
typedef pf2::Particle6D         particle;


//global variable
ros::Publisher  tracking_output;

ros::Time       five_frames_ago;                    //used to caculate frequence
int             time_counter = 0;
int             downsample_flag = 1;

double          dbscan_eps = 0.3;                   //Distance between two points       dbscan
int             dbscan_min = 4;                     //Minimum number of a cluster       dbscan

double          valid_area_x_min = 0.5;            //Used to exclude clusters outside of this area
double          valid_area_x_max = 7.0;
double          valid_area_y_min = -5.0;
double          valid_area_y_max = 5.0;

double          aspect_ratio_threshold = 7;         //Used to exclude clusters which does not meet this aspect ratio
int             trajectory_length = 200;            //can be considered as the length of the sequence
double          nearest_neighbor_threshold = 0.2;
int             lost_time_threshold = 100;          //how long we keep this object if there is no match scan, the unit is frame
int             trajectory_length_threshold = 10;   //to determine whether this cluster is a noise
double          identical_distance_threshold = 0.3;
double          similarity_threshold = 0.95;
int             uwb_lost_time = 0;
int             generate_particle_time_threshold = 10;
int             num_particle_samples = 10;
double          particle_update_noise = 0.05;

std::string     plot_min_x = "0";                   //plot aera
std::string     plot_max_x = "8";
std::string     plot_min_y = "-5";
std::string     plot_max_y = "5";

typedef struct 
{
    double x;
    double y;
    int label;                          //clustering result
}LidarPoint;

typedef struct
{
    double x;
    double y;
    double vx;
    double vy;
    double range;
    bool is_valid = true;
    //int pointNum;
}ObjectState;

typedef struct
{
    double range;
    double rx_rssi;
    double fp_rssi;
}UwbState;

class LidarCluster
{
public:
    int id;
    double aspect_ratio;
    bool is_associated;                 // Whether this cluster is associated to an object
    std::vector<LidarPoint> points;
    LidarPoint center_coordinate;
    LidarCluster()
    {
        points.clear();
        id = -1;
        aspect_ratio = INFINITY;
        is_associated = false;
    }
};

class Object
{
public:
    ObjectState now_state;
    ObjectState last_valid_state;
    ObjectState last_3_valid_state;
    ParticleFilter *searching_area;
    int id;
    int lost_time;
    int matched_uwb_id;
    int matched_time;
    bool searching_area_is_created;
    bool is_lost;
    bool is_updated;
    bool is_moving;
    std::vector<LidarPoint> points;
    double area;
    double girth;
    double width;
    double depth;
    boost::circular_buffer<ObjectState> trajectory;
    Object()
    {
        id = -1;
        lost_time = 10000;
        trajectory.resize(trajectory_length);
        trajectory.clear();
        is_lost = true;
        is_updated = false;
        is_moving = false;
        matched_uwb_id = -1;
        matched_time = 0;
        girth = 0;
        area = 0;
        width = 0;
        depth = 0;
        searching_area_is_created = false;
        points.clear();
        //searching_area->clear();
    }
    bool getMovingStatus()
    {
        double mean_x = 0, mean_y = 0, count = 0;
        for (boost::circular_buffer<ObjectState>::iterator it_his = this->trajectory.begin(); it_his != this->trajectory.end(); ++it_his)
        {
            mean_x += it_his->x;
            mean_y += it_his->y;
            count++;
        }
        mean_x /= count;
        mean_y /= count;

        double range = 0, dx = 0, dy = 0;
        dx = this->now_state.x - mean_x;
        dy = this->now_state.y - mean_y;
        range = sqrt(carmen_square(dx) + carmen_square(dy));
        if (range > 0.5)
            this->is_moving = true;
        else
            this->is_moving = false;

        return this->is_moving;
    }
    void getFeatures()
    {
        this->width = sqrt(carmen_square(this->points.front().x - this->points.back().x) + carmen_square(this->points.front().y - this->points.back().y));

        double a, b, c, s, max_d = 0;
        c = this->width;
        for (std::vector<LidarPoint>::iterator it_point = this->points.begin() + 1; it_point != this->points.end(); ++it_point)
        {
            this->girth += sqrt(carmen_square(it_point->x - (it_point - 1)->x) + carmen_square(it_point->y - (it_point - 1)->y));
            a = sqrt(carmen_square(this->points.front().x - it_point->x) + carmen_square(this->points.front().y - it_point->y));
            b = sqrt(carmen_square(this->points.back().x - it_point->x) + carmen_square(this->points.back().y - it_point->y));
            s = 0.5 * (a + b + c);
            double d = (2 * sqrt(s * (s - a) * (s - b) * (s - c))) / c;
            if (d > max_d)
                max_d = d;
        }

        this->depth = max_d;
        this->area = this->depth * this->width;
    }
};



class UwbNode
{
public:
    int id;
    UwbState now_state;
    bool is_updated;
    int lost_time;
    int matched_obj_id;
    boost::circular_buffer<UwbState>    history;
    boost::circular_buffer<ObjectState> trajectory;
    UwbNode()
    {
        history.resize(trajectory_length);
        history.clear();
        trajectory.resize(trajectory_length);
        trajectory.clear();
        is_updated = false;
        lost_time = 10000;
        matched_obj_id = -1;
    }
};

//global arrays
std::vector<int> object_ids;                        //Record which ids have been used ,to prevent identical ids from appearing
std::vector<LidarCluster> clusters;
std::vector<Object> objects;
std::vector<UwbNode> uwbs;
std::map<int,int> uwb_obj_match_record;

//gnuplot
GnuplotInterface *plot_clusters = new GnuplotInterface();
GnuplotInterface *plot_objects = new GnuplotInterface();
GnuplotInterface *plot_uwbs = new GnuplotInterface();
GnuplotInterface *plot_fusion_result = new GnuplotInterface();

//iterators
typedef std::vector<LidarCluster>::iterator ClusterIterator;
typedef std::vector<Object>::iterator ObjectIterator;
typedef std::vector<nlink_parser::LinktrackNode2>::iterator UwbIterator;
typedef std::vector<UwbNode>::iterator UwbNodeIterator;
typedef std::vector<LidarPoint>::iterator LidarPointIterator;

//function declaration
void plotLidarCluster(std::vector<LidarCluster> &input_clusters);
void plotObjects(std::vector<Object> &input_objects);
void plotUwbHistory(std::vector<UwbNode> &input_uwbs);