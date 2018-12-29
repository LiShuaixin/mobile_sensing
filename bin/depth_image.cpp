/**
 * \func 可视化点云数据投影得到的深度图像
 */

#include <system.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main (int argc, char ** argv)
{
    Mobile_Sensing::System mobile_sensing(argc, argv, Mobile_Sensing::HDL_64);
    
    std::cout << "System begin to run...\n" << std::endl;
    
    int poses_size = mobile_sensing.span_reader_->size(), scans_size = mobile_sensing.velodyne_reader_->size(), frame_num, map_save_num; 
    
    if(mobile_sensing.frame_num_ == 0)
        frame_num = scans_size;
    else
        frame_num = mobile_sensing.frame_num_;
    
    // if poses_size = scans_size -> kitti dataset, the scan and pose are one-to-one correspondence
    if(poses_size == scans_size)
    {
        for(int i = 0; i < frame_num; i++)
	{
	    SPANPose pose = mobile_sensing.span_reader_->read_kitti(i);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr points = mobile_sensing.velodyne_reader_->read_pointcloud(i);
	    long long lidar_start_time = mobile_sensing.velodyne_reader_->read_timestamp(i);
	    
	    // create Scan object
	    Scan scan(points, lidar_start_time, mobile_sensing.projection_params_);
	    
	    mobile_sensing.mapper_->depth_ground_remover_->process(scan);    
	    
	    std::cout << "Mapping " << i << "th frame kitti data..." << std::endl;
	    
	}
    }

    return (0);
}