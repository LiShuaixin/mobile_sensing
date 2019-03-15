#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <vector>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdio.h>  
#include <stdlib.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>      
#include <pcl/io/pcd_io.h>                    
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <typeinfo>

#include <registration.h>
#include <datareader.h>
#include <spanpose.h>
#include <projection_params.h>
#include <map_drawer.h>
#include <frame_drawer.h>
#include <viewer.h>
#include <mapping.h>
#include <scan.h>
#include <local_map.h>

namespace Mobile_Sensing
{
class SPANCPTReader;
class VelodyneReader;
class FrameDrawer;
class MapDrawer;
class Viewer;
class Mapping;
class ProjectionParams;
class Registration;
class LocalMap;

enum LiDAR { VLP_16 = 0, HDL_32 = 1, HDL_64 = 2, HDL_64_EQUAL = 3, VELO_CONFIG = 4, FUll_SPHERE = 5 };

class System
{
    
public:
    System(int argc, char** argv, const LiDAR lidar_type, const bool use_viewer = true);
    
    void run();
    
    void shutdown();
    
    void save_map( pcl::PointCloud<pcl::PointXYZI>::ConstPtr coarse_map, const int& index );
    
    // boost::mutex scan_load_mutex_;
    
public:
    std::string pts_filepath_;
    std::string pts_timestamps_file_;
    std::string pos_filepath_;
    std::string pos_timestamps_file_;
    
    LiDAR lidar_type_;
    
    ProjectionParams* projection_params_;
    
    bool use_viewer_;
    
    int frame_num_;
    int map_save_num_;
    
    SPANCPTReader* span_reader_;
    VelodyneReader* velodyne_reader_;
    
    Viewer* viewer_;
    FrameDrawer* frame_drawer_;
    MapDrawer* map_drawer_;
    
    Mapping* mapper_;
    
    LocalMap* local_mapper_;
    
    Registration* registrator_;
    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    boost::thread* local_mapping_thread_; // 局部优化线程
    boost::thread* loop_closing_thread_; // 闭环线程
    boost::thread* viewer_thread_; // 可视化线程
    
    // Mapping state 状态互斥锁
    // boost::mutex mapping_state_mutex_; 
};

  
}

#endif