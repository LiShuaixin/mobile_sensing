#include <datareader.h>

namespace Mobile_Sensing
{
  
SPANCPTReader::SPANCPTReader ( const string& poses_filepath, const string& poses_timestamps_file , bool is_kitti_ ) : 
               txt_filepath_(poses_filepath), 
               txt_timestamps_file_(poses_timestamps_file) 
{  
    Ro_ = Eigen::Matrix3f::Identity();
    to_ = Eigen::Vector3f::Zero();
    
    // read kitti GPS/INS data
    ifstream ftime;
    std::string filepath(txt_filepath_);
    std::string timestamps(txt_timestamps_file_);
    
    ftime.open(timestamps.c_str());
    
    while(!ftime.eof())
    {
        std::string s;
	std::getline(ftime, s);
	if(s.empty())
	{
	    std::cout << "Load SPAN failed!" << std::endl;
	}
	
	long long stamp;
	if(is_kitti_)
	    stamp = parse_kitti_stamp(s);
	else
	    stamp = std::atof(s.c_str());
	
	v_timestamps_.push_back(stamp);
    }
    
    const int num_times = v_timestamps_.size();
    vstr_filename_.resize(num_times);
    
    for(int i = 0; i < num_times; i++)
    {
        char tmp[256];
	std::sprintf(tmp, "%010d.txt", i);
	std::string filename(tmp);
	
	std::string full_txt_filename = filepath + filename;
	
	vstr_filename_[i] = full_txt_filename;
    }
}


SPANPose SPANCPTReader::read( const int& index )
{   
    SPANPose pose;
    
    long long stamp = v_timestamps_[index];
    std::string filename( vstr_filename_[index]);
 
    ifstream fpos;
    fpos.open(filename.c_str());
    
    std::string s;
    std::getline(fpos, s);
    
    if(s.empty())
    {
	std::cout << "Load SPAN failed!" << std::endl;
    }
    
    std::string lat, lon, alt, ro, pi, ya;
    istringstream is(s);
    is >> lat >> lon >> alt >> ro >> pi >> ya;
    
    Eigen::Vector3d BLH(atof(lat.c_str())*M_PI/180, atof(lon.c_str())*M_PI/180, atof(alt.c_str()));

    if(index == 0)
    {
	scale_ = cos(BLH(0));
	to_ = trans_blh_to_xyz(BLH, scale_);
	ro_ = atof(ro.c_str()); po_ = atof(pi.c_str()); ho_ = atof(ya.c_str());
	Ro_ = trans_euler_to_rotation3(ro_, po_, ho_);
    }
    
    Eigen::Vector3f twc = trans_blh_to_xyz(BLH, scale_); 

    // 旋转角: 由ENU到当前时刻各轴系的旋转角度
    float roll, pitch, heading;
    roll = atof(ro.c_str());
    pitch = atof(pi.c_str());
    heading = atof(ya.c_str());
    Eigen::Matrix3f Rwc = trans_euler_to_rotation3(roll, pitch, heading); 
	
    Eigen::Matrix4f Tmp = Eigen::Matrix4f::Identity(), Twc, T0;
    T0 = trans_matrix(Ro_, to_);

    Tmp.topLeftCorner<3, 3>() = Rwc;
    Tmp.topRightCorner<3, 1>() = twc;	    
    Twc = T0.inverse() * Tmp;

    pose.roll_ = roll;
    pose.pitch_ = pitch;
    pose.heading_ = heading;
    pose.timestamp_ = stamp;
    pose.twc_ = Twc.topRightCorner<3, 1>();
    pose.Rwc_ = Twc.topLeftCorner<3, 3>();
    
    pose_ = pose;
    
    fpos.close();
    
    return pose;
}

SPANPose SPANCPTReader::read_kitti( const int& index )
{  
    SPANPose pose;
    
    long long stamp = v_timestamps_[index];
    std::string filename( vstr_filename_[index]);
 
    ifstream fpos;
    fpos.open(filename.c_str());
    
    std::string s;
    std::getline(fpos, s);
    
    if(s.empty())
    {
	std::cout << "Load Velodyne failed!" << std::endl;
    }
    
    std::string lat, lon, alt, ro, pi, ya, vn, ve, vf, vl, vu, 
		ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu, 
		pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode;

    istringstream is(s);
    is >> lat >> lon >> alt >> ro >> pi >> ya >> vn >> ve >> vf >> vl >> vu >> 
	  ax >> ay >> az >> af >> al >> au >> wx >> wy >> wz >> wf >> wl >> wu >> 
	  pos_accuracy >> vel_accuracy >> navstat >> numsats >> posmode >> velmode >> orimode;
    
    Eigen::Vector3d BLH(atof(lat.c_str())*M_PI/180, atof(lon.c_str())*M_PI/180, atof(alt.c_str()));
    
    if(index == 0)
    {
	scale_ = cos(BLH(0));
	to_ = trans_blh_to_xyz(BLH, scale_);
	ro_ = atof(ro.c_str()); po_ = atof(pi.c_str()); ho_ = atof(ya.c_str());
	Ro_ = trans_euler_to_rotation3(ro_, po_, ho_);
    }
    
    Eigen::Vector3f twc = trans_blh_to_xyz(BLH, scale_); 
    
    // 旋转角: 由ENU到当前时刻各轴系的旋转角度
    float roll, pitch, heading;
    roll = atof(ro.c_str());
    pitch = atof(pi.c_str());
    heading = atof(ya.c_str());
    Eigen::Matrix3f Rwc = trans_euler_to_rotation3(roll, pitch, heading); 
    
    Eigen::Matrix4f Tmp = Eigen::Matrix4f::Identity(), Twc, T0;
    T0 = trans_matrix(Ro_, to_);
    
    Tmp.topLeftCorner<3, 3>() = Rwc;
    Tmp.topRightCorner<3, 1>() = twc;	    
    Twc = T0.inverse() * Tmp;
 
    pose.roll_ = roll;
    pose.pitch_ = pitch;
    pose.heading_ = heading;
    pose.timestamp_ = stamp;
    pose.twc_ = Twc.topRightCorner<3, 1>();
    pose.Rwc_ = Twc.topLeftCorner<3, 3>();
    
    pose_ = pose;
    
    fpos.close();
    
    return pose;
}

long long SPANCPTReader::unixtime_offset( const std::string& y, const std::string& m, const std::string& d )
{
    int year = atoi(y.c_str()), month = atoi(m.c_str()), day = atoi(d.c_str());
    
    int offset_year = ((year-1970)*365 + (int)((year-1972)/4 + 1));
    // std::cout << "year offset = " << (int)((year-1972)/4 + 1) << std::endl;
    
    int offset_month;
    int n;
    if(month == 1)
	n = 0;
    if(month == 2 || month == 3)
	n = 1;
    if(month == 4 || month == 5)
	n = 2;
    if(month == 6 || month == 7)
	n = 3;
    if(month == 8)
	n = 4;
    if(month == 9 || month == 10)
	n = 5;
    if(month > 10)
	n = 6;
	    
    if(year % 4 == 0)
	offset_month = (month-1)*30 - 1 + n;
    else
	offset_month = (month-1)*30 - 2 + n;
    
    int offset_day = day - 1;
    
    long long offset = (offset_year + offset_month + offset_day) * 24 * 60 * 60 * 1e6;
    // std::cout << "unixtime offset = " << offset << " ms." << std::endl;
    
    return offset;
}

VelodyneReader::VelodyneReader ( const string& pts_filepath, const std::string& pts_timestamps_file, bool is_kitti_ ) : 
                pts_filepath_(pts_filepath), 
                pts_timestamps_file_(pts_timestamps_file), 
                scan_(new pcl::PointCloud<pcl::PointXYZI>())
{
    ifstream ftime;
    std::string filepath(pts_filepath_); // filepath of .bin files
    std::string timestamps(pts_timestamps_file_);
    
    ftime.open(timestamps.c_str());
    
    while(!ftime.eof())
    {
	std::string s;
	std::getline(ftime, s);
	if(s.empty())
	{
	    std::cout << "Load Velodyne failed!" << std::endl;
	}
	
	long long stamp;
	if(is_kitti_)
	    stamp = parse_kitti_stamp(s);
	else
	    stamp = std::atof(s.c_str());
	
	v_timestamps_.push_back(stamp);
    }
    
    const int num_times = v_timestamps_.size();
    vstr_filename_.resize(num_times);
    
    for(int i = 0; i < num_times; i++)
    {
        char tmp[256];
	std::sprintf(tmp, "%010d.bin", i);
	std::string filename(tmp);
	
	std::string full_bin_filename = filepath + filename;
	
	vstr_filename_[i] = full_bin_filename;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr VelodyneReader::read_pointcloud( const int& index )
{  
    /* read file */
    FILE *binary_file;
    
    /* allocate 4 MB buffer */
    int32_t num_points = 1000000; // the number of point clouds
    float *data = (float*)malloc(num_points * sizeof(float));
    
    /* pointers */
    float *px = data + 0; // point to the first entry of data
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;
    
    std::string curr_binary_filename = vstr_filename_[index];
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pts(new pcl::PointCloud<pcl::PointXYZI>());
    
    if((binary_file = std::fopen(curr_binary_filename.c_str(), "rb")) == NULL )
    {
	std::cout << "Load " << index << "th binary file failed!" << std::endl;
    }
    num_points = std::fread(data, sizeof(float), num_points, binary_file)/4; // read file from binary_file file and save in the data pointer
    
    for(int32_t i = 0; i < num_points; i++)
    {
	pcl::PointXYZI point;
	point.x = *px;
	point.y = *py;
	point.z = *pz;
	point.intensity = *pr;
	
	pts->push_back(point);
	
	px += 4;
	py += 4;
	pz += 4;
	pr += 4;
    }
    std::fclose(binary_file);
    
    //从点云中移除NAN点也就是无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pts,*pts, indices);
    
    scan_ = pts;
    
    return pts;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr VelodyneReader::read_scan ( const int& index )
{  
    // Scan cur_scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_scan(new pcl::PointCloud<pcl::PointXYZI>());
       
    std::string path = vstr_filename_[index];
    
    std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
    
    if (file.good()) 
    {
	file.seekg(0, std::ios::beg);
	float intensity = 0;
	for (int i = 0; file.good() && !file.eof(); ++i) 
	{
	    // RichPoint point;
	    pcl::PointXYZI point;
	    file.read(reinterpret_cast<char*>(&point.x), sizeof(float));
	    file.read(reinterpret_cast<char*>(&point.y), sizeof(float));
	    file.read(reinterpret_cast<char*>(&point.z), sizeof(float));
	    file.read(reinterpret_cast<char*>(&point.intensity), sizeof(float));
	    cur_scan->push_back(point);
	}
	file.close();
    }
    
    scan_ = cur_scan;
    
    return cur_scan;
}

long long int VelodyneReader::read_timestamp ( const int& index )
{
    long long stamp = v_timestamps_[index];
    
    return stamp;
}

}