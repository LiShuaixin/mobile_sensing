#include <scan.h>
#include <thread>

namespace Mobile_Sensing
{
    
Scan::Scan () : 
      cloud_ (new pcl::PointCloud<pcl::PointXYZI>()), 
      Rwc_(Eigen::Matrix3f::Identity()), 
      twc_(Eigen::Vector3f::Zero()), start_time_(0) {}

Scan::Scan(const Scan& scan) : 
      Rwc_(scan.Rwc_), twc_(scan.twc_), 
      cloud_(scan.cloud_), start_time_(scan.start_time_), 
      timestamp_(scan.timestamp_), azimuth_(scan.azimuth_), 
      vertical_(scan.vertical_), mask_(scan.mask_), 
      scan_projection_(scan.scan_projection_) {}

Scan::Scan(PointCloud::Ptr velo_cloud, const long long& timestamp, ProjectionParams* proj_params) : 
      cloud_(velo_cloud), 
      Rwc_(Eigen::Matrix3f::Identity()), 
      twc_(Eigen::Vector3f::Zero()), 
      start_time_(timestamp) 
{
    scan_projection_ = new ScanProjection( proj_params );
    
    removenan();

    calculate_real_stamps();
    
    Eigen::Matrix4f T0 = Eigen::Matrix4f::Identity();
    set_pose( T0 );
    
    initial_projection();
}

void Scan::set ( Scan& scan )
{
    cloud_ = scan.cloud_;	    
    start_time_ = scan.start_time_;
    timestamp_ = scan.timestamp_; 
    azimuth_ = scan.azimuth_; 
    vertical_ = scan.vertical_;
    scan_projection_ = scan.scan_projection_;
    
    Rwc_ = scan.Rwc_;
    twc_ = scan.twc_;
}

void Scan::set_start_time( long long int& start_time )
{
    start_time_ = start_time;
}

void Scan::set_pose( const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& twc )
{
    Rwc_ = Eigen::Matrix3f(Rwc);
    twc_ = Eigen::Vector3f(twc);
}

void Scan::set_pose ( const Eigen::Matrix4f& Twc )
{
    Rwc_ = Eigen::Matrix3f(Twc.topLeftCorner<3, 3>());
    twc_ = Eigen::Vector3f(Twc.topRightCorner<3, 1>());
}

void Scan::search_pose(const std::vector<SPANPose> poses)
{
    std::vector<long long> stamps = timestamp_; // time stamps for each point
    
    //-- find corresponding poses between pos and start/end point
    int start_stamp_id, end_stamp_id;
    float diff1, diff2;
    for(int id_pose = 0; id_pose < poses.size(); id_pose++)
    {
        diff1 = (stamps[0] - poses[id_pose].timestamp_) / 1e6; // s
	diff2 = (stamps[stamps.size()-1] - poses[id_pose].timestamp_) / 1e6; // s
	// std::cout << "diff1 = " << std::abs(diff1) << ", diff2 = " << std::abs(diff2) << std::endl;
	
	if( std::abs(std::abs(diff1) -0.15) < 0.1 )
	{
	    start_stamp_id = id_pose; 
	    
	    if(std::abs(std::abs(diff2) -0.15) >= 0.1){
	        id_pose++;
		diff2 = (stamps[poses.size()-1] - poses[id_pose].timestamp_) / 1e6;
	    }
	    
	    end_stamp_id = id_pose;
	    
	    break;
	}
    }
    // std::cout << "first point corresponding to the " << start_stamp_id << "th pose" << std::endl;
    // std::cout << "last point corresponding to the " << end_stamp_id << "th pose" << std::endl;

    Rwc_ = poses[start_stamp_id].Rwc_;
    twc_ = poses[start_stamp_id].twc_;

    start_stamp_id++;
}

void Scan::removenan()
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_,*cloud_, indices);
}

// BUG fix the bug
void Scan::calculate_real_stamps()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts = cloud_;
    int cloud_size = pts->points.size();
    long long start_time = start_time_;
    // std::cout << "\nstart time = " << start_time << std::endl;
    std::vector<long long> point_time_stamp;
    
    float start_ori = -atan2(pts->points[0].y, pts->points[0].x);
    float end_ori = -atan2(pts->points[cloud_size - 1].y, pts->points[cloud_size - 1].x) + 2 * M_PI;
    
    // 确定end、ori的水平面位置关系->输入的点云坐标系为前x左y上z
    if(end_ori - start_ori > 3 * M_PI){
	    end_ori -= 2 * M_PI;
    }else if(end_ori - start_ori < M_PI){
	    end_ori += 2 * M_PI;
    }
    
    bool halfPassed = false;
    int count = cloud_size;
    
    for(int i = 0; i < cloud_size; i++)
    {	    
	float ori = -atan2(pts->points[i].y, pts->points[i].x);
	if(!halfPassed)
	{
	    if(ori < start_ori-M_PI/2)
		ori += 2 * M_PI;
	    else if(ori < start_ori + M_PI*3/2)
		ori -= 2 * M_PI;
	    
	    if(ori - start_ori > M_PI)
		halfPassed = true;
	}
	else
	{
	    ori += 2* M_PI;
	    
	    if(ori < end_ori - M_PI*3/2)
		ori += 2 * M_PI;
	    else if(ori > end_ori + M_PI/2)
		ori -= 2 * M_PI;
	}
	
	// 每帧点云中各点扫描时间
	long long real_time = start_time + ((ori - start_ori) / (end_ori - start_ori)) * 0.1 * 1e6;
	// if(i == 0 || i == cloud_size-1)
	    // std::cout << "real time = " << real_time << std::endl;
	point_time_stamp.push_back( real_time );	    
    }
    
    timestamp_ = point_time_stamp;
}

void Scan::initial_projection()
{
    scan_projection_->init_from_points(cloud_);
}

}
