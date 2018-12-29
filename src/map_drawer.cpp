#include <map_drawer.h>

namespace Mobile_Sensing
{

MapDrawer::MapDrawer()
{
    graph_line_width_ = 2.;
    point_size_ = 2.;
    lidar_size_ = 0.5;
    lidar_line_width_ = 2.;
    trajectory_.reserve(8000);
}

void MapDrawer::update ( Mapping *mapper )
{
    boost::mutex::scoped_lock lock(lidar_mutex_);
    cur_scan_ = Scan(mapper->cur_scan_);
    trajectory_.push_back(cur_scan_.get_pose());
}

void MapDrawer::draw_map_points()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>()), tmp(new pcl::PointCloud<pcl::PointXYZI>());
    tmp = cur_scan_.cloud_;
    Eigen::Matrix4f trans = cur_scan_.get_pose();
    std::list<size_t> mask = cur_scan_.mask_;

    pcl::transformPointCloud(*tmp, *points, trans);
    if(points->empty())
        return;

    glPointSize(point_size_);
    glBegin(GL_POINTS);
    
    std::list<size_t>::const_iterator iter_mask = mask.begin();
    for(size_t i = 0, iend = points->size(); i < iend; i++)
    {
        pcl::PointXYZI point = points->points[i];
	
	float intensity = point.intensity;
	if(i == *iter_mask){
	    glColor3f(intensity,intensity*5,intensity);
	    iter_mask++;
	}
	else
	    glColor3f(intensity,intensity,intensity);
	
        glVertex3f(point.x, point.y, point.z);
    } 
    glEnd();
}

void MapDrawer::draw_current_lidar ( pangolin::OpenGlMatrix& Twc )
{
    
    const float &w = lidar_size_;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

    // set transformation matrix
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(lidar_line_width_);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    
    // NOTE opengl coordinate: upward-z forward-x left-y
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,-z);
    glVertex3f(0,0,0);
    glVertex3f(w,h,-z);

    glVertex3f(w,h,z);
    glVertex3f(w,h,-z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(w,-h,z);
    glVertex3f(w,-h,-z);

    glVertex3f(w,-h,-z);
    glVertex3f(w,h,-z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::draw_trajectory()
{
    auto trajectory = trajectory_;

    glLineWidth(graph_line_width_);
    glColor4f(1.0f,0.0f,0.0f,0.6f);
    glBegin(GL_LINES);

    for(size_t i=1; i<trajectory.size(); i++)
    {
	// trajectory
	Eigen::Matrix4f pose1 = trajectory[i-1];
	Eigen::Matrix4f pose2 = trajectory[i];
	Eigen::Vector3f Ow1 = pose1.topRightCorner<3, 1>();
	Eigen::Vector3f Ow2 = pose2.topRightCorner<3, 1>();
	
	glVertex3f(Ow1(0),Ow1(1),Ow1(2));
	glVertex3f(Ow2(0),Ow2(1),Ow2(2));
    }

    glEnd();
}

void MapDrawer::get_current_OpenGL_lidar_matrix ( pangolin::OpenGlMatrix& M )
{
    Eigen::Matrix3f Rwc;
    Eigen::Vector3f twc;
    {
	boost::mutex::scoped_lock lock(lidar_mutex_);;
	Rwc = cur_scan_.Rwc_;
	twc = cur_scan_.twc_;
    }
    
    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;
}

} //namespace Mobile_Sensing
