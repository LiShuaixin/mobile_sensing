#include <mapping.h>

namespace Mobile_Sensing
{

Mapping::Mapping( System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer, ProjectionParams* projection_params ) : 
         map_(new pcl::PointCloud<pcl::PointXYZI>()), 
         system_(system), 
         mapping_state_(NOT_INITIALIZED),
         frame_drawer_(frame_drawer),
         map_drawer_(map_drawer),
         viewer_(NULL),
         projection_params_(projection_params)
{   
    // kitti 04, 10
    Eigen::Matrix3f Rlc;
    Rlc <<  9.999976e-01, 7.553071e-04, -2.035826e-03,
           -7.854027e-04, 9.998898e-01, -1.482298e-02, 
	    2.024406e-03, 1.482454e-02,  9.998881e-01;
    Eigen::Vector3f tlc( -8.086759e-01, 3.195559e-01, -7.997231e-01 );
    
    Rcl_ = Rlc.inverse();
    tcl_ = - Rcl_ * tlc;
    
    /*Eigen::Matrix3f R;
    R << -1,  0, 0,
          0, -1, 0,
	  0,  0, 1;
	  
    Rcl_ = R * trans_euler_to_rotation( (90.1555)*M_PI/180, (-0.1401)*M_PI/180, (-45.1418)*M_PI/180 );
    tcl_ << -0.0057, -0.1206, 0.0432;*/
    
    // create depth_ground_remover
    int smooth_window_size = 9;
    float ground_remove_angle = 7.;
    
    depth_ground_remover_ = new DepthGroundRemover( *projection_params_, ground_remove_angle, smooth_window_size );
}

// relative transformation from src to tgt
Eigen::Matrix4f Mapping::get_relative_transformation_guess( Scan& src, Scan& tgt )
{
    Eigen::Matrix4f T_tgt = tgt.get_pose();
    Eigen::Matrix4f T_src = src.get_pose();
    
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    
    T = T_tgt.inverse() * T_src;
    
    return T;
}

void Mapping::update_scan_pose(Scan& src, Scan& tgt, const Eigen::Matrix4f& T_update)
{   
    Eigen::Matrix4f T_last = src.get_pose();
    
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    
    T = T_last * T_update;
    
    // Update the member
    tgt.set_pose(T);
}

void Mapping::update_scan_pose (SPANPose& pose, Scan& scan )
{
    
    float diff = ( pose.timestamp_ - scan.start_time_ ) / 1e6; // s 
    // std::cout << "diff between pose and scan = " << diff << std::endl;
    
    //-- interpolate the euler angle and translation for the start/end point 
    float sroll, spitch, sheading;
    sroll = angular_velocity_(0) * diff;
    spitch = angular_velocity_(1) * diff;
    sheading = angular_velocity_(2) * diff;
    // std::cout << "angular_velocity_ = \n" << angular_velocity_ << std::endl;
    
    Eigen::Vector3f st;
    st = translation_velocity_ * diff;
    // std::cout << "translation_velocity_ = \n" << translation_velocity_ << std::endl;
    
    // WARNING Rotation matrix is different according to the defintion of output yaw/pitch/roll
    Eigen::Matrix3f Ro = trans_euler_to_rotation3(sroll, spitch, sheading); // Li(start)->POS(i)
    Eigen::Vector3f to = pose.Rwc_.inverse() * st; // Li(start)->POS(i) in POS(i)
    // std::cout << "Ro = \n" << Ro << std::endl;
    // std::cout << "to = \n" << to << std::endl;
    
    Eigen::Matrix4f dT = Eigen::Matrix4f::Identity(), T = Eigen::Matrix4f::Identity(), Twc = Eigen::Matrix4f::Identity();
    dT.topLeftCorner<3, 3>() = Ro;  
    dT.topRightCorner<3, 1>() = to; // Li(start)->POS(i)
    Twc.topLeftCorner<3, 3>() = pose.Rwc_;  
    Twc.topRightCorner<3, 1>() = pose.twc_; // Li(start)->POS(i)
    // std::cout << "dT = \n" << dT << std::endl;
    // std::cout << "pose = \n" << trans_matrix( pose.Rwc_, pose.twc_ ) << std::endl;
    T = Twc * dT; // Twl = Twi * Til
    // std::cout << "Lidar trans_matrix in update func = \n" << T << std::endl;
    
    scan.set_pose(T);
}

void Mapping::align_scans( Scan& src_scan, Scan& tgt_scan )
{   
    PointCloud::Ptr src_cloud = src_scan.get_cloud();
    PointCloud::Ptr tgt_cloud = tgt_scan.get_cloud();
    
    int is_downsize = (src_cloud->size() > 10000? 1:0);
    
    float downsize_scale;
    if(is_downsize)
        downsize_scale = (src_cloud->size() > 20000? 0.5:0.3);
    
    // compute the transformation from src to tgt
    PointCloud::Ptr tgt(new pcl::PointCloud<PointT>());
    PointCloud::Ptr src(new pcl::PointCloud<PointT>());
   
    // downsize of point
    if(is_downsize)
    {
	pcl::VoxelGrid<PointT> downsize_filter;
	downsize_filter.setLeafSize(downsize_scale, downsize_scale, downsize_scale);

	downsize_filter.setInputCloud(src_cloud);
	downsize_filter.filter(*src);
	downsize_filter.setInputCloud(tgt_cloud);
	downsize_filter.filter(*tgt);
    }
    else
    {
	src = src_cloud;
	tgt = tgt_cloud;
    }

    pcl::GeneralizedIterativeClosestPoint4D reg(0.05);
    // pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    // pcl::IterativeClosestPointNonLinear<PointT, PointT> reg;
    
    reg.setTransformationEpsilon(1e-4); 
    
    reg.setMaxCorrespondenceDistance(0.5);    
    reg.setInputSource(src);
    reg.setInputTarget(tgt);
    
    Eigen::Matrix4f prev = Eigen::Matrix4f::Identity(), Ti, guess, incr = Eigen::Matrix4f::Identity();

    // transform point cloud according to the guess transformation
    PointCloud::Ptr reg_result(new PointCloud);
    guess = get_relative_transformation_guess(src_scan, tgt_scan);
    // std::cout << "    transformation guess =\n " << guess << std::endl;
    
    Ti = guess; // Ti is the updated transformation from current pose to the last pose
    pcl::transformPointCloud(*src, *reg_result, guess);
    
    
    reg.setMaximumIterations(2);
    
    for(int i = 0; i < 15; i++)
    {
	src = reg_result;
	reg.setInputSource(src);
	reg.align(*reg_result);
	
	Ti = reg.getFinalTransformation() * Ti; // The full transformation from current_scan(src)->last_scan(tgt)
	incr = reg.getFinalTransformation() * incr;
	
	if( fabs((reg.getLastIncrementalTransformation()-prev).sum())<reg.getTransformationEpsilon() )
	{
	    reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance()-0.05);
	}
	
	prev = reg.getLastIncrementalTransformation(); 
    }
    
    // std::cout << "Updated incremental transformation =\n " << incr << std::endl;
    // std::cout << "    Updated Lidar trans_matrix =\n " << Ti << std::endl;
    // std::cout << "    has converged:" << reg.hasConverged() << " score: " << reg.getFitnessScore() << std::endl;
    
    // calculate the updated transformation T(Li->W)
    update_scan_pose(last_scan_, src_scan, Ti);
}

void Mapping::joint_scans( Scan& scan )
{    
    pcl::PointCloud<pcl::PointXYZI>::Ptr global(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr local(new pcl::PointCloud<pcl::PointXYZI>());
    local = scan.cloud_;
    
    Eigen::Matrix4f Twc = trans_matrix( scan.Rwc_, scan.twc_ );
    
    Eigen::Matrix4f Tcl = trans_matrix( Rcl_, tcl_ );
    
    Eigen::Matrix4f Twl = Twc * Tcl;
    
    pcl::transformPointCloud(*local, *global, Twl);
    // std:: cout << "transform the point cloud" << std::endl;
    
    map_->resize(map_->size() + global->size());
    *map_ += *global;
    // std::cout << "expand the map" << std::endl;
}

void Mapping::deskew ( Scan& scan )
{
    int scan_size = scan.get_cloud()->size();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts = scan.get_cloud();
    std::vector<long long> stamps = scan.timestamp_; // time stamps for each point
    
    //-- interpolate the euler angle and translation for each point
    float scan_period = (stamps[scan_size-1] - stamps[0]) / 1e6;
    for(int id_pt = 1; id_pt < scan_size; id_pt++)
    {
        //-- interpolate the euler angle
       float droll, dpitch, dheading;
       float diff = (stamps[0] - stamps[id_pt]) / 1e6;
       
       // 计算start->cur的旋转角
       droll = angular_velocity_(0) * diff;
       dpitch = angular_velocity_(1) * diff;
       dheading = angular_velocity_(2) * diff;
       
       // WARNING Rotation matrix is different according to the defintion of output yaw/pitch/roll
       Eigen::Matrix3f dR = trans_euler_to_rotation3(droll, dpitch, dheading); // Li(i)->Li(start)
       
       //-- interpolate the translation
       Eigen::Vector3f dt = cur_pose_.Rwc_.inverse() * ( translation_velocity_ * diff ); // Li(i)->Li(start) in Li(start)
       // std::cout << std::fixed << "twc = [" << twc(0) << ", " << twc(1) << ", " << twc(2) << "]." << std::endl;
       
       pcl::PointXYZI pt = pts->points[id_pt];
       Eigen::Vector3f p(pt.x, pt.y, pt.z);
       
       Eigen::Vector3f q, tmp;
       q = dR * p + dt;
       
       /* 比较deckew的作用 */
       /*if(id_pt == 1)
	  std::cout << "Original pt =\n" << p << "\n, transformed pt = \n" << q << std::endl; */
       
       // transform all points to the start point
       pts->points[id_pt].x = q(0);
       pts->points[id_pt].y = q(1);
       pts->points[id_pt].z = q(2);

    }
    
    scan.cloud_.swap(pts);
}

void Mapping::initialization ( SPANPose& pose, Scan& scan )
{   
    // set current frame data   
    cur_scan_ = Scan(scan);
    cur_pose_ = SPANPose(pose);
    
    // preprocess raw pointcloud
    depth_ground_remover_->process(cur_scan_);
    
    // set angular velocity and trans velocity
    angular_velocity_ = Eigen::Vector3f::Zero();
    translation_velocity_ = Eigen::Vector3f::Zero();
    trans_velocity_ = Eigen::Matrix4f::Identity();
    
    // update the scan pose
    update_scan_pose( cur_pose_, cur_scan_ );
	
    // set last frame data
    last_scan_ = Scan(cur_scan_);
    last_pose_ = SPANPose(cur_pose_);
  
    mapping_state_ = INITIALIZED;
}

void Mapping::set_viewer ( Viewer* viewer )
{
    viewer_ = viewer;
}

void Mapping::run ( SPANPose& pose, Scan& scan  )
{   
    // run
    if(mapping_state_ == NOT_INITIALIZED)
    {
        initialization( pose, scan );
	// std::cout << "initialized" << std::endl;
	
	// update viewer 
        frame_drawer_->update(this);
        map_drawer_->update(this);
    }
    else
    {
	// set current frame data
	cur_scan_ = Scan(scan);
        cur_pose_ = SPANPose(pose);
	// std::cout << "Set current pose" << std::endl;
	
	// preprocess raw pointcloud
        depth_ground_remover_->process(cur_scan_);
	// std::cout << "Preprocess scan" << std::endl;
	
	// set angular velocity and trans velocity if pose changes
	if(cur_pose_.timestamp_ != last_pose_.timestamp_)
	{
	    float dt = (cur_pose_.timestamp_ - last_pose_.timestamp_) / 1e6;
	    float wr = (cur_pose_.roll_- last_pose_.roll_) / dt;
	    float wp = (cur_pose_.pitch_- last_pose_.pitch_) / dt;
	    float wy = (cur_pose_.heading_- last_pose_.heading_) / dt;
	    angular_velocity_ << wr, wp, wy;
	    
	    translation_velocity_ = (cur_pose_.twc_ - last_pose_.twc_) / dt;

	    // WARNING Rotation matrix is different according to the defintion of output yaw/pitch/roll
	    trans_velocity_.topLeftCorner<3, 3>() = trans_euler_to_rotation3( wr*dt, wp*dt, wy*dt ); // relative transformation (Last->current)
	    trans_velocity_.topRightCorner<3, 1>() = cur_pose_.Rwc_.inverse() * (translation_velocity_*dt); // Last->current in current
	}
	// std::cout << "Update velocity" << std::endl;
	
	// 1. Update the scan pose according to the pose input
	update_scan_pose( cur_pose_, cur_scan_ );
	// std::cout << "Update scan pose" << std::endl;
	
	// 2. Deskew the scan according to the motion model
	// deskew( cur_scan_ );
	// std::cout << "Deskew" << std::endl;
	
	// 3. Align current scan to with the last to optimized the updated scan pose
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	
	align_scans(cur_scan_, last_scan_);
	
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
	cout<<"aligning scans costs time: "<<time_used.count()<<" seconds."<<endl;
	
	// 4. visualize
	frame_drawer_->update(this);
	// std::cout << "Update frame drawer" << std::endl;
	map_drawer_->update(this);
	// std::cout << "Update map drawer" << std::endl;
	
	// set last frame data
	last_scan_ = Scan(cur_scan_);
        last_pose_ = SPANPose(cur_pose_);
	// std::cout << "Set last pose" << std::endl;
    }
    
    // 5. Joint all scans with updated poses to the global map
    joint_scans(cur_scan_);
    // std::cout << "Joint scan" << std::endl;
}

}
