#ifndef MAPPING_H
#define MAPPING_H  

#include <time.h>
#include <chrono>

#include <pcl/common/time.h> //fps calculations
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/registration/icp.h>           
#include <pcl/registration/icp_nl.h>        
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/transforms.h>     

#include <datareader.h>
#include <scan.h>
#include <spanpose.h>
#include <mcgicpIntensity.h>
#include <system.h>
#include <utils.h>
#include <projection_params.h>
#include <depth_ground_remover.h>
#include <frame_drawer.h>
#include <map_drawer.h>
#include <viewer.h>

using namespace std;
using namespace pcl;

namespace Mobile_Sensing{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class System;
class ProjectionParams;
class DepthGroundRemover;
class FrameDrawer;
class MapDrawer;
class Viewer;

class Mapping
{
        
public:
    
    enum MappingState
    {
        INITIALIZED = 1,
	NOT_INITIALIZED = 0
    };
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef Cloud::Ptr CloudPtr;
    
    Mapping ( System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer, ProjectionParams* projection_params );
    
    void run ( SPANPose& pose, Scan& scan );
    
    void initialization( SPANPose& pose, Scan& scan );
    
    void align_scans(Scan& src, Scan& tgt);
    
    Eigen::Matrix4f get_relative_transformation_guess( Scan& src, Scan& tgt);
    
    void update_scan_pose(Scan& src, Scan& tgt, const Eigen::Matrix4f& T_update);
    void update_scan_pose(SPANPose& pose, Scan& scan);
    
    void joint_scans( Scan& scan );
    
    void deskew( Scan& scan ); 
    
    void set_viewer( Viewer* viewer );
    /**
     * @brief preprocess raw pointcloud including projecting to the image, removing ground and labeling etc.
     *
     * @param[in] Scan current scan with raw pointcloud
     * 
     * @param[out] Scan current scan with preprocessed pointcloud  
     */
    void preprocess( Scan& scan );
    
    inline Scan get_current_scan() { return cur_scan_; }
    inline Scan get_last_scan() { return last_scan_; }
    inline SPANPose get_current_pose() { return cur_pose_; }
    inline SPANPose get_last_pose() { return last_pose_; }

public:
    
    MappingState mapping_state_;
    
    ProjectionParams* projection_params_;
    
    DepthGroundRemover* depth_ground_remover_;
    
    // TODO pointcloud segmentation
    // ImageBasedClusterer<LinearImageLabeler<>> clusterer( angle_tollerance, min_cluster_size, max_cluster_size );
    // clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

    // System
    System* system_;
    
    //Drawers
    Viewer* viewer_;
    FrameDrawer* frame_drawer_;
    MapDrawer* map_drawer_;
    
    boost::mutex scan_mutex_;
    // boost::mutex image_mutex_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
    
    Eigen::Matrix3f Rcl_;
    Eigen::Vector3f tcl_;
    
    Scan cur_scan_;
    Scan last_scan_;
    
    SPANPose cur_pose_;
    SPANPose last_pose_;
    
    Eigen::Matrix4f trans_velocity_; 
    Eigen::Vector3f angular_velocity_; // last to current
    Eigen::Vector3f translation_velocity_; // last to current
};

}

#endif //MAPPING_H
