#include <mapping.h>
#include <termios.h>

using namespace std;

void MappingProcess::init(const ros::NodeHandle& nh)
{
    nh_ = nh;
    nh_.param("mapping/origin_x", origin_(0), -3.0);
    nh_.param("mapping/origin_y", origin_(1), -2.0);
    nh_.param("mapping/map_size_x", map_size_(0), 6.0);
    nh_.param("mapping/map_size_y", map_size_(1), 4.0);
    nh_.param("mapping/resolution", resolution_, 0.05);

    resolution_inv_ = 1 / resolution_;
    for(int i=0;i<2;i++) 
    {
        global_map_size_(i) = ceil(map_size_(i) / resolution_);
    }
    cost_map_sub_ = nh_.subscribe("/local_costmap/costmap/costmap", 1, &MappingProcess::costMapCallback, this);
    live_map_sub_ = nh_.subscribe("/local_costmap/costmap/costmap_updates",10, &MappingProcess::liveMapCallback,this);
    buffer_size_2d_ = global_map_size_(0) * global_map_size_(1);
    occupancy_buffer_2d_.resize(buffer_size_2d_);
    fill(occupancy_buffer_2d_.begin(), occupancy_buffer_2d_.end(), -1);
    costmap_initialized_ = false;
}


 
void MappingProcess::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    if (map_msg->info.width != global_map_size_(0) || 
        map_msg->info.height != global_map_size_(1)) 
    {
        ROS_WARN_THROTTLE(1.0, "Dimension mismatch. Expected %dx%d, got %dx%d",
                         global_map_size_(0), global_map_size_(1),
                         map_msg->info.width, map_msg->info.height);
        return;
    }

    //copy full costmap data
    for (size_t i = 0; i < map_msg->data.size(); ++i) {
        occupancy_buffer_2d_[i] = map_msg->data[i];
    }
    costmap_initialized_ = true;
    ROS_INFO("Full costmap initialized");
}

void MappingProcess::liveMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& update_msg)
{
    if (!costmap_initialized_) {
        ROS_WARN_THROTTLE(1.0, "Received update before initial costmap");
        return;
    }

    //check update bounds
    if (update_msg->x + update_msg->width > global_map_size_(0) ||
        update_msg->y + update_msg->height > global_map_size_(1)) 
    {
        ROS_WARN("Update out of bounds: (%d,%d) size (%d,%d)",
                update_msg->x, update_msg->y,
                update_msg->width, update_msg->height);
        return;
    }

    //apply partial update
    for (int y = 0; y < update_msg->height; ++y) {
        for (int x = 0; x < update_msg->width; ++x) {
            int idx = (update_msg->y + y) * global_map_size_(0) + (update_msg->x + x);
            int update_idx = y * update_msg->width + x;
            occupancy_buffer_2d_[idx] = update_msg->data[update_idx];
        }
    }
}

bool MappingProcess::isInMap2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i idx;
    posToIndex2d(pos, idx);
    return isInMap2d(idx);
}

bool MappingProcess::isInMap2d(const Eigen::Vector2i &id)
{
    if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
    {
        return false;
    }
    else
        return true;
};

void MappingProcess::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
{
    for(int i = 0; i < 2; i++)
        id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void MappingProcess::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
{
    for(int i = 0; i < 2; i++){
        pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
        std::cout << floor((pos(i) - origin_(i)) * resolution_inv_);
    }
}

int MappingProcess::getVoxelState2d(const Eigen::Vector2d &pos)
{
    Eigen::Vector2i id;
    posToIndex2d(pos, id);
    if(!isInMap2d(id))
        return -1;
    return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)] > 50 ? 1 : 0;
}
