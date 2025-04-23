#include "uwb_lidar_tracking.hpp"

// DBSCAN 算法
//  计算该点周围满足距离要求的点的数量 并把满足距离要求的 点 在input中的序号存入到output->seeds中
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
// dbscan ends

// lidar pre-processing
std::vector<LidarPoint> lidarPreprocessing(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    vector<LidarPoint> points;
    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
        LidarPoint temp_point;
        double angle = 0, range = 0;
        if (!std::isfinite(scan_msg->ranges[i])) // std::isfinite ：Returns true if the value entered is a valid value
        {
            continue;
        }
        angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        range = scan_msg->ranges[i];
        temp_point.x = range * cos(angle);
        temp_point.y = range * sin(angle);
        points.push_back(temp_point);
    }
    return points;
}

// to determine if the clustering is valid
bool clusterValidation(LidarCluster &cluster)
{
    bool flag_valid_1 = false;
    bool flag_valid_2 = false;
    bool flag_valid = false;

    // Determine if the position is valid
    if (cluster.center_coordinate.x > valid_area_x_min && cluster.center_coordinate.x < valid_area_x_max && cluster.center_coordinate.y > valid_area_y_min && cluster.center_coordinate.y < valid_area_y_max)
    {
        flag_valid_1 = true;
    }
    // Determine if the aspect ratio is valid
    if (cluster.aspect_ratio < aspect_ratio_threshold)
        flag_valid_2 = true;

    if (flag_valid_1 && flag_valid_2)
        flag_valid = true;

    return flag_valid;
}

std::vector<LidarCluster> getClusters(std::vector<LidarPoint> &points)
{
    std::vector<LidarCluster> clusters;
    int clusters_num = dbscan(points, dbscan_eps, dbscan_min);
    int valid_id = 1;
    for (int i = 1; i <= clusters_num; i++)
    {
        std::vector<LidarPoint>::iterator it_point;
        LidarCluster temp_cluster;
        std::vector<LidarPoint> points_set_temp;
        double total_x = 0;
        double total_y = 0;
        double max_x = -INFINITY, max_y = -INFINITY, min_x = INFINITY, min_y = INFINITY;

        for (it_point = points.begin(); it_point != points.end(); it_point++)
        {
            if (it_point->label == i)
            {
                if (it_point->x > max_x)
                    max_x = it_point->x;
                if (it_point->y > max_y)
                    max_y = it_point->y;
                if (it_point->x < min_x)
                    min_x = it_point->x;
                if (it_point->y < min_y)
                    min_y = it_point->y;
                total_x = total_x + it_point->x;
                total_y = total_y + it_point->y;
                points_set_temp.push_back(*it_point);
            }
        }

        temp_cluster.points = points_set_temp;
        temp_cluster.center_coordinate.x = total_x / (double)points_set_temp.size();
        temp_cluster.center_coordinate.y = total_y / (double)points_set_temp.size();
        double temp_aspect_ratio;
        temp_aspect_ratio = (fabs(max_x - min_x)) / (fabs(max_y - min_y));
        if (temp_aspect_ratio < 1)
            temp_aspect_ratio = 1.0 / temp_aspect_ratio;
        temp_cluster.aspect_ratio = temp_aspect_ratio;

        // Removing redundant clusters from the environment
        if (clusterValidation(temp_cluster))
        {
            temp_cluster.id = valid_id;
            for (LidarPointIterator it_point = temp_cluster.points.begin(); it_point != temp_cluster.points.end(); ++it_point)
            {
                it_point->label = valid_id;
            }
            clusters.push_back(temp_cluster);
            valid_id++;
        }
    }
    if(flag_plot_clusters)
        plotLidarCluster(clusters);
    return clusters;
}

int generateRandomID()
{
    int objID = rand() % 100 + 1;
    if (find(object_ids.begin(), object_ids.end(), objID) == object_ids.end())
        return objID;
    else
    {
        objID = generateRandomID();
        return objID;
    }
}

// used to sort a map
bool compareValue(const std::pair<int, double> &a, const std::pair<int, double> &b)
{
    return a.second < b.second;
}

Object objectInit(LidarCluster &input_cluster)
{
    Object new_object;
    new_object.id = generateRandomID();
    new_object.now_state.x = input_cluster.center_coordinate.x;
    new_object.now_state.y = input_cluster.center_coordinate.y;
    new_object.now_state.vx = 0;
    new_object.now_state.vy = 0;
    new_object.now_state.range = sqrt(carmen_square(new_object.now_state.x) + carmen_square(new_object.now_state.y));
    new_object.lost_time = 0;
    new_object.is_lost = false;
    new_object.is_updated = false;
    input_cluster.is_associated = true;

    return new_object;
}

// The current scan finds the nearest neighbor match for the object ,so we use cluster information to update this object
Object objectUpdate(LidarCluster &input_cluster, Object &input_object)
{
    Object object_update;
    object_update = input_object;
    object_update.lost_time = 0;
    object_update.is_lost = false;
    input_object.is_updated = true;
    object_update.is_updated = false;
    double last_x, last_y;
    last_x = object_update.now_state.x;
    last_y = object_update.now_state.y;
    object_update.trajectory.push_back(object_update.now_state);
    object_update.now_state.x = input_cluster.center_coordinate.x;
    object_update.now_state.y = input_cluster.center_coordinate.y;
    object_update.now_state.vx = object_update.now_state.x - last_x;
    object_update.now_state.vy = object_update.now_state.y - last_y;
    object_update.now_state.range = sqrt(carmen_square(object_update.now_state.x) + carmen_square(object_update.now_state.y));
    input_cluster.is_associated = true;

    return object_update;
}

Object objectUpdate(Object &input_object)
{
    Object object_update;
    input_object.is_updated = true;
    if (input_object.lost_time < lost_time_threshold && (int)input_object.trajectory.size() > trajectory_length_threshold)
    // The nearest neighbor match for the object is not scanned, so we use the previous state to keep it for a while
    {
        object_update = input_object;
        object_update.last_valid_state = input_object.now_state;
        object_update.lost_time++;
        object_update.now_state.vx = 0;
        object_update.now_state.vy = 0;
        object_update.is_lost = true;
        object_update.trajectory.push_back(object_update.now_state);
    }
    else
    // we delete this object
    {
        object_update = input_object;
        object_update.trajectory.clear();
        object_update.id = -1;
    }

    return object_update;
}

void clustersToObjects(std::vector<LidarCluster> &clusters)
{
    std::vector<Object> new_objects;
    new_objects.clear();
    if (objects.empty())
    {
        if (!clusters.empty())
        {
            for (ClusterIterator it_cluster = clusters.begin(); it_cluster != clusters.end(); ++it_cluster)
            {
                new_objects.push_back(objectInit(*it_cluster));
            }
        }
    }
    else if (!clusters.empty())
    {
        // 1.we update objects
        for (ClusterIterator it_cluster = clusters.begin(); it_cluster != clusters.end(); ++it_cluster)
        {
            double nn_distance = 100000;
            int nn_object_id = -1;
            for (ObjectIterator it_object = objects.begin(); it_object != objects.end(); ++it_object)
            {
                double distance = sqrt(carmen_square(it_cluster->center_coordinate.x - it_object->now_state.x) +
                                       carmen_square(it_cluster->center_coordinate.y - it_object->now_state.y));
                if (distance < nn_distance & distance < nearest_neighbor_threshold)
                {
                    nn_object_id = it_object->id;
                    nn_distance = distance;
                }
            }
            if (nn_object_id < 0)
                continue;

            for (ObjectIterator it_object = objects.begin(); it_object != objects.end(); ++it_object)
            {
                if (it_object->id == nn_object_id && nn_object_id > 0 && it_cluster->is_associated == false)
                {
                    new_objects.push_back(objectUpdate(*it_cluster, *it_object));
                    break;
                }
            }
        }
        // 2.we handle the rest of objects
        for (ObjectIterator it_object = objects.begin(); it_object != objects.end(); ++it_object)
        {
            if (!it_object->is_updated)
            {
                Object temp_object = objectUpdate(*it_object);
                if (temp_object.id > 0)
                {
                    new_objects.push_back(temp_object);
                }
            }
        }

        // 3. we handle the rest of clusters
        for (ClusterIterator it_cluster = clusters.begin(); it_cluster != clusters.end(); ++it_cluster)
        {
            if (!it_cluster->is_associated)
            {
                new_objects.push_back(objectInit(*it_cluster));
            }
        }
    }
    objects.clear();
    objects = new_objects;
    if(flag_plot_objects)
        plotObjects(new_objects);

    if(flag_object_debug)
    {
        for (ObjectIterator it_object = objects.begin(); it_object != objects.end(); ++it_object)
        {
            std::cout<< "object id: " << it_object->id << "\n";
            for(boost::circular_buffer<ObjectState>::iterator it_obj_his = it_object->trajectory.begin();
                it_obj_his!=it_object->trajectory.end(); ++it_obj_his)
            {
                std::cout<< it_obj_his->x <<" "<<it_obj_his->y << "\n";
            }
            std::cout << std::endl;
        }
    }
}

void uwbNodeInit(nlink_parser::LinktrackNode2 &linktrack_node,std::vector<UwbNode> &uwb_set)
{
    UwbNode uwb_temp;
    uwb_temp.id = linktrack_node.id;
    uwb_temp.now_state.range = linktrack_node.dis;
    uwb_temp.now_state.fp_rssi = linktrack_node.fp_rssi;
    uwb_temp.now_state.rx_rssi = linktrack_node.rx_rssi;
    uwb_temp.is_updated = false;
    uwb_temp.lost_time = 0;
    uwb_set.push_back(uwb_temp);
}

bool uwbNodeUpdate(nlink_parser::LinktrackNode2 &linktrack_node, std::vector<UwbNode> &uwb_set)
{
    
    bool update_status = false;
    if (!uwbs.empty())
    {
        UwbNode uwb_temp;
        for (UwbNodeIterator it_uwb_node = uwbs.begin(); it_uwb_node != uwbs.end(); ++it_uwb_node)
        {
            if (it_uwb_node->id == linktrack_node.id)
            {
                uwb_temp = *it_uwb_node;
                uwb_temp.history.push_back(uwb_temp.now_state);
                uwb_temp.now_state.range = linktrack_node.dis;
                uwb_temp.now_state.fp_rssi = linktrack_node.fp_rssi;
                uwb_temp.now_state.rx_rssi = linktrack_node.rx_rssi;
                uwb_temp.is_updated = false;
                it_uwb_node->is_updated = true;

                update_status = true;
                break;
            }
        }
        if(update_status)
        {
            uwb_set.push_back(uwb_temp);
        }
    }

    return update_status;
}

void uwbPreprocessing(const nlink_parser::LinktrackNodeframe2::ConstPtr &range_msg)
{
    std::vector<nlink_parser::LinktrackNode2> uwb_nodes = range_msg->nodes;
    std::vector<UwbNode> new_uwbs;

    if(!uwb_nodes.empty() && uwb_lost_time < 5)
    {
        for (UwbIterator it_uwb = uwb_nodes.begin(); it_uwb != uwb_nodes.end(); ++it_uwb)
        {
            if (!uwbNodeUpdate(*it_uwb,new_uwbs))
                uwbNodeInit(*it_uwb,new_uwbs);
        }
        for (UwbNodeIterator it_uwb_node = uwbs.begin(); it_uwb_node != uwbs.end(); ++it_uwb_node)
        {
            if (it_uwb_node->is_updated = false && it_uwb_node->lost_time < lost_time_threshold)
            {
                UwbNode uwb_temp;
                uwb_temp = *it_uwb_node;
                uwb_temp.lost_time++;
                new_uwbs.push_back(uwb_temp);

                it_uwb_node->is_updated = true;
            }
        }
        uwbs.clear();
        uwbs = new_uwbs;
        if(flag_plot_uwb_history)
            plotUwbHistory(new_uwbs);
        uwb_lost_time = 0;
    }
    else if(uwb_nodes.empty())      //there comes no new uwb message, so we clean vector uwbs after a while
    {
        for (UwbNodeIterator it_uwb_node = uwbs.begin(); it_uwb_node != uwbs.end(); ++it_uwb_node)
        {
            it_uwb_node->history.push_back(it_uwb_node->now_state);
            it_uwb_node->is_updated = false;
            it_uwb_node->lost_time++;
        }
        uwb_lost_time ++;
    }

    if(flag_uwb_debug)
    {
        for(UwbNodeIterator it_uwb = uwbs.begin(); it_uwb!= uwbs.end(); ++it_uwb)
        {
            std::cout << "id r iu lt hs " << it_uwb->id << " " << it_uwb->now_state.range << " " << it_uwb->is_updated << " " << it_uwb->lost_time << " " << it_uwb->history.size() << std::endl;
        }
    }
}

double getJaccardFactor(Object &input_obj, UwbNode &input_uwb)
{
    if(input_obj.trajectory.size()< 5 || input_uwb.history.size()<5)
        return 0.0;
    // Intercept the same length of the segment, the length is the smaller of the two input value
    int valid_size = 0;
    if ((int)input_obj.trajectory.size() < input_uwb.history.size())
        valid_size = (int)input_obj.trajectory.size();
    else
        valid_size = input_uwb.history.size();
    std::vector<UwbState> uwb_sequence;
    std::vector<ObjectState> obj_sequence;

    boost::circular_buffer<UwbState>::iterator it_uwb_s = input_uwb.history.end();
    boost::circular_buffer<ObjectState>::iterator it_obj_s = input_obj.trajectory.end();
    for (int i = 1; i <= valid_size; i++)
    {
        uwb_sequence.push_back(*(it_uwb_s - i));
        obj_sequence.push_back(*(it_obj_s - i));
    }
    reverse(uwb_sequence.begin(), uwb_sequence.end());
    reverse(obj_sequence.begin(), obj_sequence.end());

    double identical_flag = 0;
    for (int i = 0; i < valid_size; i++)
    {
        if (fabs(uwb_sequence[i].range - obj_sequence[i].range) < identical_distance_threshold)
            identical_flag = identical_flag + 1.0;
    }
    return identical_flag / (double)valid_size;
}

double getGaussianSimilarity(Object &input_obj, UwbNode &input_uwb)
{
    if(input_obj.trajectory.size()< 5 || input_uwb.history.size()<5)
        return 0.0;
    // Intercept the same length of the segment, the length is the smaller of the two input value
    int valid_size = 0;
    if ((int)input_obj.trajectory.size() < input_uwb.history.size())
        valid_size = (int)input_obj.trajectory.size();
    else
        valid_size = input_uwb.history.size();

    std::vector<UwbState> uwb_sequence;
    std::vector<ObjectState> obj_sequence;

    boost::circular_buffer<UwbState>::iterator it_uwb_s = input_uwb.history.end();
    boost::circular_buffer<ObjectState>::iterator it_obj_s = input_obj.trajectory.end();
    for (int i = 1; i <= valid_size; i++)
    {
        uwb_sequence.push_back(*(it_uwb_s - i));
        obj_sequence.push_back(*(it_obj_s - i));
    }
    reverse(uwb_sequence.begin(), uwb_sequence.end());
    reverse(obj_sequence.begin(), obj_sequence.end());

    double euclidean_distance = 0, sigma = 1.0;
    for (int i = 0; i < valid_size; i++)
    {
        euclidean_distance += carmen_square(uwb_sequence[i].range - obj_sequence[i].range);
    }
    euclidean_distance /= valid_size;
    return exp(-0.5 * euclidean_distance / carmen_square(sigma));
}

void sequenceFusion()
{
    for (UwbNodeIterator it_uwb = uwbs.begin(); it_uwb != uwbs.end(); ++it_uwb)
    {
        double max_similarity = -INFINITY;
        double max_g_s = -INFINITY;
        double max_j_f = -INFINITY;
        int matched_obj_id = -1;
        // std::vector<Object> match_candidates;
        for (ObjectIterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj)
        {
            double gaussian_similarity = getGaussianSimilarity(*it_obj, *it_uwb);
            double jaccard_factor = getJaccardFactor(*it_obj, *it_uwb);
            double similarity = gaussian_similarity * jaccard_factor;
            if (similarity > max_similarity)
            {
                matched_obj_id = it_obj->id;
                max_similarity = similarity;
            }
            if (gaussian_similarity > max_g_s)
            {
                max_g_s = gaussian_similarity;
            }
            if (jaccard_factor > max_j_f)
            {
                max_j_f = jaccard_factor;
            }
            if(flag_obj_similarity_debug)
                std::cout<< "uwbid " << it_uwb->id << " objid " <<  it_obj->id << " gaussian_similarity " << gaussian_similarity << " jaccard_factor " << jaccard_factor << " similarity " << similarity << std::endl;
        }
        if(flag_obj_similarity_debug)
            std::cout << "uwbid " << it_uwb->id << " max sim " << max_similarity << " max gaussian " << max_g_s << " max jaccard " << max_j_f << std::endl;
        for (ObjectIterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj)
        {
            if (it_obj->id == matched_obj_id)
            {
                it_uwb->matched_obj_id = it_obj->id;
                it_uwb->trajectory.push_back(it_obj->now_state);
            }
        }
    }
}

void plotLidarCluster(std::vector<LidarCluster> &input_clusters)
{
    int points_num = 0;
    std::vector<LidarPoint> points_ploting;
    for (ClusterIterator it_cluster = input_clusters.begin(); it_cluster != input_clusters.end(); ++it_cluster)
    {
        points_ploting.insert(points_ploting.end(), it_cluster->points.begin(), it_cluster->points.end());
    }
    points_num = (int)points_ploting.size();

    std::string cmd; //( "set size ratio 1\n")
    cmd += "set grid\n";
    cmd += "set xlabel 'x'\n";
    cmd += "set ylabel 'y'\n";
    //cmd += "plot [0:10][-5:5] ";
    cmd += "plot [" + plot_min_x + ":" + plot_max_x + "][" + plot_min_y + ":" + plot_max_y + "]";       //there cant be any "\n" or else it cant plot

    for (int i = 1; i <= (int)input_clusters.size(); i++)
    {
        std::string caption = "cluster " + to_string(i);
        cmd += "'-' using 1:2 w p lc " + to_string(i) + " ti '" + caption + "' ,";
    }
    cmd += "\n";
    for (int i = 1; i <= (int)input_clusters.size(); i++)
    {
        for (int j = 0; j < points_num; j++)
        {
            if (points_ploting[j].label == i)
            {
                cmd += toString(points_ploting[j].x) + ' ' + toString(points_ploting[j].y) + ' ' + to_string(0.5) + '\n';
            }
        }
        cmd += "e\n";
    }

    plot_clusters->commandStr(cmd);
}

void plotObjects(std::vector<Object> &input_objects)
{
    std::string cmd; //( "set size ratio 1\n")
    cmd += "set grid\n";
    cmd += "set xlabel 'x'\n";
    cmd += "set ylabel 'y'\n";
    //cmd += "plot [0:10][-5:5] ";
    cmd += "plot [" + plot_min_x + ":" + plot_max_x + "][" + plot_min_y + ":" + plot_max_y + "]";       //there cant be any "\n" or else it cant plot
    
    std::vector<ObjectState> points_ploting;
    points_ploting.clear();
    for(ObjectIterator it_obj = input_objects.begin();it_obj!=input_objects.end();++it_obj)
    {
        points_ploting.insert(points_ploting.end(),it_obj->now_state);
    }
    for(int i = 0; i < input_objects.size(); i++)
    {
        std::string object_id = "object " + to_string(input_objects[i].id);
        cmd += "'-' using 1:2 w p lc " + to_string(input_objects[i].id % 10) + " ti '" + object_id + "' ,";
    }
    cmd += "\n";
    for(std::vector<ObjectState>::iterator it_plot = points_ploting.begin(); it_plot!=points_ploting.end();++it_plot)
    {
        cmd += toString(it_plot->x) + ' ' + toString(it_plot->y) + ' ' + to_string(0.5) + '\n';
        cmd += "e\n";
    }
    plot_objects->commandStr(cmd);
}

void plotUwbHistory(std::vector<UwbNode> &input_uwbs)
{

    std::string cmd; //( "set size ratio 1\n")
    cmd += "set grid\n";
    cmd += "set xlabel 'x' \n";
    cmd += "set ylabel 'y' \n";
    //cmd += "set xtics 1,1,10 \n";
    cmd += "set xrange [-10:" + to_string(trajectory_length + 5) + "] \n";
    cmd += "set yrange [0:10]\n";
    cmd += "plot ";
    for(int i = 0; i< input_uwbs.size(); i++)
    {
        std::string uwb_id = "uwb " + to_string(input_uwbs[i].id);
        cmd += "'-' using 1:2 w p lc " + to_string(input_uwbs[i].id % 5) + " ti '" + uwb_id + "' ,";
    }
    for(int i = 0; i< objects.size(); i ++)
    {
        std::string object_id = "object " + to_string(objects[i].id);
        cmd += "'-' using 1:2 w p lc " + to_string(objects[i].id % 10) + " ti '" + object_id + "' ,";
    }
    cmd += "\n";
    for(int i = 0; i < input_uwbs.size() ;i ++)
    {
        for(int j = 0; j< input_uwbs[i].history.size();j = j+5)
        {
            cmd += toString(j) + ' ' + toString(input_uwbs[i].history[j].range)+ ' ' + to_string(0.5) + '\n';
        }
        cmd += "e\n";
    }
    for(int i = 0; i < objects.size(); i++)
    {
        for(int j = 0; j< objects[i].trajectory.size();j = j+5)
        {
            cmd += toString(j) + ' ' + toString(objects[i].trajectory[j].range)+ ' ' + to_string(0.5) + '\n';
        }
        cmd += "e\n";
    }
    plot_uwbs->commandStr(cmd);
}

void recordLocationResult(double &timestamp)
{
    std::ofstream uwb_file(uwb_file_name, std::ios::app);
    for (UwbNodeIterator it_uwb = uwbs.begin(); it_uwb != uwbs.end(); ++it_uwb)
    {
        uwb_file << std::to_string(it_uwb->id) << " " << std::to_string(timestamp) << " "
                 << it_uwb->trajectory.back().x << " " << it_uwb->trajectory.back().y << "\n";

        std::cout <<"file outputing : "<<std::to_string(it_uwb->id) << " " << it_uwb->trajectory.back().x << " " << it_uwb->trajectory.back().y << std::endl;
    }
    uwb_file.close();
}

void recordAmcl(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
    std::cout << "AMCL Process In" << std::endl;
    double timestamp = amcl_msg->header.stamp.toSec();
    std::ofstream amcl_file(amcl_file_name, std::ios::app);
    amcl_file << std::to_string(timestamp) << " " << std::to_string(amcl_msg->pose.pose.position.x) << " " << std::to_string(amcl_msg->pose.pose.position.y) << "\n";
    amcl_file.close();
}

void callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const nlink_parser::LinktrackNodeframe2::ConstPtr &range_msg)
{
    // TODO downsample;
    if (downsample_flag % 4 != 0)
    {
        downsample_flag++;
        return;
    }
    else
    {
        time_counter ++;
        if(time_counter %10 ==0)
        {
            ros::Time now_time = ros::Time::now();
            double time_diff = now_time.toSec() - five_frames_ago.toSec();
            double frequence = 10 / time_diff;
            std::cout << "frequence : " << frequence << std::endl;
            five_frames_ago = now_time;
            time_counter = 0;
        }
        downsample_flag = 1;
        
        // we first pre-processing lidar information ,to eliminate invalid points
        std::vector<LidarPoint> valid_points = lidarPreprocessing(scan_msg);

        // then we cluster the points
        std::vector<LidarCluster> clusters = getClusters(valid_points);

        // we associate clusters with objects
        clustersToObjects(clusters);

        // handle uwb information
        uwbPreprocessing(range_msg);

        // we associate object and uwb information with sequences
        sequenceFusion();

        if(flag_save_output)
        {
            double uwb_timestamp = range_msg->header.stamp.toSec();
            recordLocationResult(uwb_timestamp);
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_lidar_tracking");
    ros::NodeHandle nh;

    std::cout << "program started" << std::endl;

    // We synchronize the data from laser and uwb ,and then enter the same callback function
    ros::Subscriber laser_scan_subscriber;
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, lidar_scan_topic, 1);
    message_filters::Subscriber<nlink_parser::LinktrackNodeframe2> range_sub(nh, uwb_topic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nlink_parser::LinktrackNodeframe2> LidarlocationSyncPolicy;
    message_filters::Synchronizer<LidarlocationSyncPolicy> sync(LidarlocationSyncPolicy(10), scan_sub, range_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    if (flag_amcl_record)
        ros::Subscriber amcl_subscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(amcl_topic, 10, recordAmcl);

    while (nh.ok())
    {
        ros::spinOnce();
    }

    return 0;
}