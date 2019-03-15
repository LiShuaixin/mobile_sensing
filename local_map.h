#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include <string>

#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>

#include <scan.h>
#include <utils.h>
#include <mapping.h>
#include <registration.h>
#include <map_drawer.h>
#include <viewer.h>
#include <surfel_map/map_point_types.h>
#include <surfel_map/multiresolution_surfel_map.h>
#include <surfel_map/synchronized_circular_buffer.h>

/**
 * BUG 2. mrs map-> 点坐标与地图坐标系，转换关系要进一步明确
 */
namespace Mobile_Sensing
{
class Scan;
class Registration;
class Mapping;
class MapDrawer;
class Viewer;

class LocalMap
{
public:
    typedef pcl::PointXYZI MapPointType;
    typedef Mobile_Sensing::MultiResolutionalMap<MapPointType> MapType;
    
    LocalMap ( Registration* registrator, MapDrawer* map_drawer );
    
    void set_mapper( Mapping* mapper );
    
    void set_viewer( Viewer* viewer );
    
    void add_scan( Scan* scan );

    void run();
    
    // Thread Synch
    void request_stop();
    void request_reset();
    bool stop();
    void release();
    void clear();
    bool is_stopped();
    bool stop_requested();
    bool accept_add_scan();
    void set_accept_add_scan(bool flag);
    bool set_not_stop(bool flag);
    bool is_running();
    
    void interrupt_optimization();
    
    void request_finish();
    bool is_finished();
    
    int scans_in_buffer()
    {
	boost::mutex::scoped_lock lock(mutex_scans_buffer_);
	return scan_buffer_.size();
    }

    MapType* get_mrs_map() { return multiresolution_map_.get(); };
    
    // relative transformation from src to tgt
    // Eigen::Matrix4f get_relative_transformation( const Scan& src, const Scan& tgt);
        
    Scan* latest_inserted_scan_;
    Scan* last_scan_;
    Eigen::Matrix4f successive_scans_relative_transform_;
    
protected:
    bool check_new_scan();
    void register_scan( Scan* scan );
    
    void reset_if_requested();
    bool reset_requested_;
    boost::mutex mutex_reset_;
    
    bool check_finish();
    void set_finish();
    bool finish_requested_;
    bool finished_;
    boost::mutex mutex_finish_;
    
    bool stopped_;
    bool stop_requested_;
    bool not_stop_;
    boost::mutex mutex_stop_;
    
    bool running_;
    boost::mutex mutex_running_;

private:
    // threshold params
    int scan_number_;
    double decrease_rate_;
    float max_dist_between_scans_;
    int scans_in_local_map_;

    // fitness threshold
    double fitness_thr1_;
    double fitness_thr2_;
    
    // mrs map params
    int map_size_;
    int map_levels_;
    int map_cell_capacity_;
    double map_resolution_;

    int map_downsampled_size_;
    int map_downsampled_levels_;
    int map_downsampled_cell_capacity_;
    double map_downsampled_resolution_;

    // occupied cells params
    float param_clamping_thresh_min_;
    float param_clamping_thresh_max_;
    float param_prob_hit_;
    float param_prob_miss_;    
    
    // threads params
    bool add_new_scan_;
    boost::mutex mutex_accept_add;
    
    bool first_scan_;
    bool decrease_once_;
    bool abort_optimization_;

    // data
    Eigen::Matrix3f map_orientation_; // pose from the world to the pose of latest inserted lidar scan ({W}->{Ck-1})
    boost::mutex mutex_map_transform_; 
    
    boost::shared_ptr< MapType > multiresolution_map_; // multi-resolution local map with points in the {Ck-1} 
    boost::mutex mutex_local_map_;
    
    mrs_laser_maps::synchronized_circular_buffer<Scan*> scan_buffer_; // scan_buffer 缓存了1s内lidar的全部scan信息
    boost::mutex mutex_scans_buffer_;
    
    // tools
    Registration* registrator_;
    
    Mapping* mapper_;
    
    //Drawers
    Viewer* viewer_;
    MapDrawer* map_drawer_;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif