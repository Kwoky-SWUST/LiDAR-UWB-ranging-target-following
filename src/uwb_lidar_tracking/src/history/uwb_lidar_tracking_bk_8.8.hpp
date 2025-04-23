#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <vector>
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

#include "gnuplot.cpp"
#include "carmen.h"
#include "carmen.cpp"
#include "to_string.h"
#include "pf2_6d.h"

#include "nlink_parser/LinktrackNodeframe2.h"

//configuration
std::string lidar_scan_topic = "/robot1/scan";
std::string uwb_topic = "/robot1/nlink_linktrack_nodeframe2";
std::string amcl_file_name("/home/gglin/WORKSPACE/uwb_lidar_tracking/dataset/amcl_record.txt");
std::string uwb_file_name("/home/gglin/WORKSPACE/uwb_lidar_tracking/dataset/uwb_record.txt");
std::string amcl_topic = "/amcl_pose";
bool        flag_amcl_record            = false;

bool        flag_object_debug           = true;
bool        flag_uwb_debug              = false;
bool        flag_obj_similarity_debug   = false;

bool        flag_save_output            = false;

bool        flag_plot_clusters          = false;
bool        flag_plot_objects           = false;
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
double          similarity_threshold = 0.9;
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
    boost::circular_buffer<ObjectState> trajectory;
    Object()
    {
        id = -1;
        lost_time = 10000;
        trajectory.resize(trajectory_length);
        trajectory.clear();
        is_lost = true;
        is_updated = false;
        matched_uwb_id = -1;
        matched_time = 0;
        searching_area_is_created = false;
        searching_area->clear();
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