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

#include <registration.h>
#include <datareader.h>
#include <scan.h>
#include <spanpose.h>
#include <mcgicpIntensity.h>
#include <system.h>
#include <utils.h>
#include <projection_params.h>
#include <feature_extractor/depth_ground_remover.h>
#include <feature_extractor/image_based_clusterer.h>
#include <feature_extractor/feature_extractor.h>
#include <image_labelers/diff_helpers/diff_factory.h>
#include <image_labelers/linear_image_labeler.h>
#include <frame_drawer.h>
#include <map_drawer.h>
#include <viewer.h>
#include <local_map.h>

using namespace std;
using namespace pcl;
using namespace depth_clustering;

namespace Mobile_Sensing{
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class System;
class ProjectionParams;
class DepthGroundRemover;
class SmoothnessBasedExtractor;
class FrameDrawer;
class MapDrawer;
class Viewer;
class Registration;
class LocalMap;

class Mapping
{
    
public:
    
    enum MappingState
    {
        INITIALIZED = 1,
	NOT_INITIALIZED = 0,
	WRONG_REGISTRATION = 2,
	TRUE_REGISTRATION = 3
    };
    
public:  
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef Cloud::Ptr CloudPtr;
    
    Mapping ( System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer, Registration* registrator, ProjectionParams* projection_params );
    
    void run ( SPANPose& pose, Scan& scan );
    
    void initialization( SPANPose& pose, Scan& scan );
    
    void align_scans(Scan& src, Scan& tgt);
    
    void update_scan_pose(Scan& src, Scan& tgt, const Eigen::Matrix4f& T_update);
    void update_scan_pose(SPANPose& pose, Scan& scan);
    
    void joint_scans( Scan& scan );
    
    void deskew( Scan& scan ); 
    
    void set_viewer( Viewer* viewer );
    
    void set_local_mapper(LocalMap* local_mapper);
    
    bool need_add_new_scan();
    void create_new_scan();
    
    /**
     * @brief preprocess raw pointcloud including projecting to the image, removing ground and labeling etc.
     *
     * @param[in] Scan current scan with raw pointcloud
     * 
     * @param[out] Scan current scan with preprocessed pointcloud  
     */
    void preprocess( Scan& scan ); // TODO
    
    inline Scan get_current_scan() { return cur_scan_; }
    inline Scan get_last_scan() { return last_scan_; }
    inline SPANPose get_current_pose() { return cur_pose_; }
    inline SPANPose get_last_pose() { return last_pose_; }

public:
    
    MappingState mapping_state_;
    
    ProjectionParams* projection_params_;
    
    DepthGroundRemover* depth_ground_remover_;
    
    ImageBasedClusterer<LinearImageLabeler<>>* clusterer_;
    
    SmoothnessBasedExtractor* feature_extractor_;
    
    // TODO pointcloud segmentation
    // ImageBasedClusterer<LinearImageLabeler<>> clusterer( angle_tollerance, min_cluster_size, max_cluster_size );
    // clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

    // System
    System* system_;
    
    //Drawers
    Viewer* viewer_;
    FrameDrawer* frame_drawer_;
    MapDrawer* map_drawer_;
    Registration* registrator_;
    
    // Local Mapper
    LocalMap* local_mapper_;
    
    boost::condition_variable_any pose_update_condition_;
    boost::mutex registration_mutex_;
    boost::mutex scan_mutex_;
    // boost::mutex image_mutex_;
    
    int skipped_scan_num_;
    int skipped_scan_thre_;
    float max_dist_between_scans_;

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
    
    // fitness threshold
    double fitness_thr_;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif //MAPPING_H
