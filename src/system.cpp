#include <system.h>
#include <thread>
#include <iomanip>

namespace Mobile_Sensing 
{

System::System(int argc, char** argv, const LiDAR lidar_type, const bool use_viewer) :lidar_type_(lidar_type), use_viewer_(use_viewer), map_save_num_(100)
{
    // Output welcome message
    std::cout << std::endl <<
    "*************************************************************************************************************" << std::endl <<
    "*************************************************************************************************************" << std::endl <<
    "Mobile Sensing System (MSS) Copyright (C) 2018-2022" << std::endl << 
    "Shuaixin Li, University of Information Engineering, Zhengzhou, China." << std::endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
    "This is free software, and you are welcome to redistribute " << std::endl <<
    "it under certain conditions. See LICENSE.txt." << std::endl <<
    "Any problem can put the issue on github" << std::endl << 
    "or contact at: shuaixinli_md@126.com" << std::endl << 
    "*************************************************************************************************************" << std::endl <<
    "*************************************************************************************************************" << std::endl << std::endl;
    
    std::cout << "Command line info: " << std::endl;
    std::string pts_filepath, pts_timestamps_file, pos_file, pos_filepath, pos_timestamps_file, frame_num;

    pcl::console::parse_argument (argc, argv, "-pts_filepath", pts_filepath);
    pcl::console::parse_argument (argc, argv, "-pts_timestamps", pts_timestamps_file);
    pcl::console::parse_argument (argc, argv, "-pos_filepath", pos_filepath);
    pcl::console::parse_argument (argc, argv, "-pos_timestamps", pos_timestamps_file);
    pcl::console::parse_argument (argc, argv, "-frame_num", frame_num);
    
    std::cout << "    Velodyne filepath was set at: " << pts_filepath << std::endl;
    std::cout << "    Pointcloud timestamps file was set at: " << pts_timestamps_file << std::endl;
    
    pts_filepath_ = pts_filepath; 
    pts_timestamps_file_ = pts_timestamps_file; 
    
    std::cout << "Loading Velodyne scans. Please wait..." << std::endl;
    velodyne_reader_ = new VelodyneReader(pts_filepath_, pts_timestamps_file_);
    std::cout << "Velodyne scans have been loaded!\n" << std::endl;
    
    std::cout << "    GPS/INS filepath was set at: " << pos_filepath << std::endl;
    std::cout << "    GPS/INS timestamps file was set at: " << pos_timestamps_file << std::endl;
    
    pos_filepath_ = pos_filepath;
    pos_timestamps_file_ = pos_timestamps_file;
    
    std::cout << "Loading IMU/GNSS poses. Please wait..." << std::endl;
    span_reader_ = new SPANCPTReader(pos_filepath_, pos_timestamps_file_); // read kitti data
    std::cout << "IMU/GNSS poses have been loaded!\n" << std::endl;
    
    if( !frame_num.empty() )
    {
	frame_num_ = atoi(frame_num.c_str());
	std::cout << "Process [0, " << frame_num << "] frame." << std::endl;
    }
    else
    {
	frame_num_ = 0;
	std::cout << "Process all frame." << std::endl;
    }
    
    // create prejection param object
    projection_params_ = new ProjectionParams(lidar_type_);
    
    //create visualization tools
    frame_drawer_ = new FrameDrawer(projection_params_);
    map_drawer_ = new MapDrawer();
    
    // create mapping object
    mapper_ = new Mapping(this, frame_drawer_, map_drawer_, projection_params_);
    
    
    // TODO Initialize the refine thread and launch
     
    // TODO Initialize the loop closure thread and launch
    
    //Initialize the Viewer thread and launch
    if(use_viewer_)
    {
        viewer_ = new Viewer( this, frame_drawer_, map_drawer_, mapper_ );
        viewer_thread_ = new boost::thread( &Viewer::visualize, viewer_ );
        mapper_->set_viewer(viewer_);
    }
}

void System::save_map( pcl::PointCloud<pcl::PointXYZI>::ConstPtr coarse_map, const int& index )
{
    // map_->setInputCloud(coarse_map);
    
    // Save Map
    std::string name("map");
    std::string full_num = name + std::to_string(index) + std::string(".ply");
    
    pcl::io::savePLYFile(full_num, *coarse_map, true);
    
    // release the map
    mapper_->map_->clear();
}

void System::shutdown()
{
    if(use_viewer_)
    {
        viewer_->request_finish();
        while(!viewer_->is_finished())
            usleep(5000);
    }

    if(viewer_)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::run()
{

    std::cout << "System begin to run...\n" << std::endl;
   
    int poses_size = span_reader_->size(), scans_size = velodyne_reader_->size(), frame_num, map_save_num; 
    
    map_save_num = map_save_num_;
    
    if(frame_num_ == 0)
        frame_num = scans_size;
    else
        frame_num = frame_num_;
    
    // if poses_size = scans_size -> kitti dataset, the scan and pose are one-to-one correspondence
    if(poses_size == scans_size)
    {
        for(int i = 0; i < frame_num; i++)
	{
	    SPANPose pose = span_reader_->read_kitti(i);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr points = velodyne_reader_->read_pointcloud(i);
	    long long lidar_start_time = velodyne_reader_->read_timestamp(i);
	    
	    // create Scan object
	    Scan scan(points, lidar_start_time, projection_params_);
	    
	    mapper_->run(pose, scan);
	    std::cout << "Mapping " << i << "th frame kitti data..." << std::endl;
	    
	    if((i != 0) && (i % map_save_num == 0))
	    {
	        // Save the map
                pcl::PointCloud<pcl::PointXYZI>::ConstPtr coarse_map(new pcl::PointCloud<pcl::PointXYZI>);
                coarse_map = mapper_->map_;
                
		std::cout << "saving local map, please wait..." << std::endl;
		save_map( coarse_map, int(i/map_save_num) );
	    }
	}
    }
    // else -> need to find the nearest pose for each scan
    else
    {
        int iter_count = 0;
        for(int i = 0; i < frame_num; i++)
	{
	    pcl::PointCloud<pcl::PointXYZI>::Ptr points = velodyne_reader_->read_pointcloud(i);
	    long long lidar_start_time = velodyne_reader_->read_timestamp(i);
	    
	    // create Scan object
	    Scan scan(points, lidar_start_time, projection_params_);
	    
	    for(; iter_count < poses_size; iter_count++)
	    {
		SPANPose pose = span_reader_->read_kitti(iter_count);
	        double d1 = (lidar_start_time-pose.timestamp_)/1e6;
		double d2 = (lidar_start_time-pose.timestamp_)/1e6;
	        if( (d1 > 0) && (d2 < 0) )
		{
		    mapper_->run(pose, scan);
		    std::cout << "Mapping " << i << "th frame data..." << std::endl;
		    
		    break;
		}
	    }
	    
	    if((i != 0) && (i % map_save_num == 0))
	    {
	        // Save the map
                pcl::PointCloud<pcl::PointXYZI>::ConstPtr coarse_map(new pcl::PointCloud<pcl::PointXYZI>);
                coarse_map = mapper_->map_;
                
		std::cout << "saving local map, please wait..." << std::endl;
		save_map( coarse_map, int(i/map_save_num) );
	    }
	}
    }
    
    // Save the map
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr coarse_map(new pcl::PointCloud<pcl::PointXYZI>);
    coarse_map = mapper_->map_;
    save_map( coarse_map, int(frame_num/map_save_num)+1 );
}

}