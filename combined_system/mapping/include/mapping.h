#ifndef _MAPPING_H
#define _MAPPING_H

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/impl/transforms.hpp>

#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
// #include "nodelet/nodelet.h"

#include <queue>

#define INVALID_IDX -1

class MappingProcess
{
public:
    MappingProcess() {};
    ~MappingProcess() {}; 
    void init(const ros::NodeHandle& nh);
    typedef std::shared_ptr<MappingProcess> Ptr;

    double getResolution() {  return resolution_;  }
    int getVoxelState2d(const Eigen::Vector2d &pos);
    int getVoxelState2d(const Eigen::Vector2i &id);
    bool isInMap2d(const Eigen::Vector2i &pos);
    bool isInMap2d(const Eigen::Vector2d &id);
    void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
    void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
    

    void setObstacle(const Eigen::Vector2d& pos) {
      Eigen::Vector2i idx;
      posToIndex2d(pos, idx);
      if (isInMap2d(idx)) {
        occupancy_buffer_2d_[idx(1) * global_map_size_(0) + idx(0)] = 100;
      }  
    }

    void clearObstacle(const Eigen::Vector2d& pos) {
      Eigen::Vector2i idx;
      posToIndex2d(pos, idx);
      if (isInMap2d(idx)) {
        occupancy_buffer_2d_[idx(1) * global_map_size_(0) + idx(0)] = 0;
      }
    }

    
private:
    ros::NodeHandle nh_;
    ros::Subscriber live_map_sub_;
    ros::Subscriber cost_map_sub_;
    Eigen::Vector2d origin_, map_size_;
    Eigen::Vector2i global_map_size_;
    double resolution_, resolution_inv_;
    std::vector<int> occupancy_buffer_2d_;
    int buffer_size_2d_;
    bool costmap_initialized_;
    void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void liveMapCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& map_msg);




};

#endif