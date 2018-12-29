#ifndef DATA_READER_H
#define DATA_READER_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <string>
#include <stdio.h>  
#include <stdlib.h>  
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>  

#include <spanpose.h>
#include <utils.h>
#include <rich_point.h>

using namespace std;

namespace Mobile_Sensing
{
  
class SPANCPTReader
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    SPANCPTReader(const std::string& poses_filepath, const std::string& poses_timestamps_file, bool is_kitti_ = true); // default: read KITTI .bin file
    ~SPANCPTReader() {};
    
    SPANPose read( const int& index );
    
    SPANPose read_kitti( const int& index );
    
    long long unixtime_offset( const std::string& y, const std::string& m, const std::string& d );
    
    inline SPANPose get_data() { return pose_; }
    
    inline int size() { return v_timestamps_.size(); };
    
public:
  
    std::string txt_filepath_;
    std::string txt_timestamps_file_;
    
    std::vector<std::string> vstr_filename_;
    std::vector<long long> v_timestamps_;
    
    SPANPose pose_; 
    
    // header info
    float GPS_offset_;
    std::string date_;
    long long unixtime_offset_;
    
    // reference coordinate param
    float ro_, po_, ho_;
    Eigen::Vector3f to_;
    Eigen::Matrix3f Ro_;
    // Eigen::Matrix4f To_;
    
    double scale_;
    
    bool is_kitti_;
};

class VelodyneReader
{
public:
    VelodyneReader(const std::string& pts_filepath, const std::string& pts_timestamps_file, bool is_kitti_ = true);
    ~VelodyneReader() {};
    
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr get_data() { return scan_; }
    
    inline int size() { return v_timestamps_.size(); };
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr read_pointcloud( const int& index );
    pcl::PointCloud<pcl::PointXYZI>::Ptr read_scan( const int& index );
    long long read_timestamp( const int& index );
    
public:
    std::string pts_filepath_;
    std::string pts_timestamps_file_;
    
    std::vector<std::string> vstr_filename_;
    std::vector<long long> v_timestamps_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_;
    
    bool is_kitti_;
};

}
#endif //DATA_READER_H