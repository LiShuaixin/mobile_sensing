#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <list>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mapping.h>
#include <utils.h>

#include <pangolin/pangolin.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

namespace Mobile_Sensing
{
    
class Mapping;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MapDrawer();
    // MapDrawer( const Scan& scan );

    // list<Scan> scans_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > trajectory_;
    
    void update( Mapping *mapper );
    void draw_map_points();
    void draw_current_lidar(pangolin::OpenGlMatrix &Twc);
    void draw_trajectory();
    void get_current_OpenGL_lidar_matrix(pangolin::OpenGlMatrix &M);
    // void draw_cube(); for segmentation

private:

    float graph_line_width_;
    float point_size_;
    float lidar_size_;
    float lidar_line_width_;

    Scan cur_scan_;

    boost::mutex lidar_mutex_;
};

} //namespace Mobile_Sensing

#endif // MAPDRAWER_H
