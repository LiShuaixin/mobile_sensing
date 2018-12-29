#include <utils.h>

namespace Mobile_Sensing 
{

long long int parse_kitti_stamp ( const std::string& s )
{
    // std::cout << "string timestamp is: " << s << std::endl;
    float hour = atof(s.substr(11, 2).c_str()); 
    float min = atof(s.substr(14, 2).c_str());
    float sec = atof(s.substr(17).c_str());
    // std::cout << "hour is: " << hour << ", minute is: " << min << ", second is: " << sec << std::endl;
    
    long long timestamp = (min * 60 + sec) * 1e6;
    // std::cout << "timestamp (num) is: " << timestamp << ", timestamp (string) is: " << str << std::endl;
    return timestamp;
}

long long parse_stamp(const std::string& s)
{
    // std::cout << "string timestamp is: " << s << std::endl;
    float hour = atof(s.substr(0, 1).c_str()); 
    float min = atof(s.substr(2, 2).c_str());
    float sec = atof(s.substr(5).c_str());
    // std::cout << "hour is: " << hour << ", minute is: " << min << ", second is: " << sec << std::endl;
    
    // float timestamp = hour * 3600 + min * 60 + sec;
    long long timestamp = (min * 60 + sec) * 1e6;
    // std::cout << "UTC time: " << timestamp << " s." << std::endl;

    return timestamp;
}

Eigen::Matrix3f trans_euler_to_rotation(const float& phi, const float& theta, const float& psi)
{
    float cphi = cos(phi), sphi = sin(phi);
    float ctheta = cos(theta), stheta = sin(theta);
    float cpsi = cos(psi), spsi = sin(psi);
    
    Eigen::Matrix3f Rz, Rx, Ry, R;
    Rz << cpsi, -spsi, 0,
	  spsi,  cpsi, 0,
	      0,     0, 1;
    Rx << 1,       0,      0,
	  0,  ctheta, stheta,
	  0, -stheta, ctheta;
    Ry << cphi, 0, -sphi,
	      0, 1,     0,
	  sphi, 0,  cphi;
    
    R = Ry * Rx * Rz;
	  
    return R;
}

Eigen::Matrix3f trans_euler_to_rotation2 ( const float& phi, const float& theta, const float& psi )
{
    float cphi = cos(phi), sphi = sin(phi);
    float ctheta = cos(theta), stheta = sin(theta);
    float cpsi = cos(psi), spsi = sin(psi);
    
    Eigen::Matrix3f Rz, Rx, Ry, R;
    Rz << cpsi, -spsi, 0,
	  spsi,  cpsi, 0,
	     0,     0, 1;
    Ry << ctheta, 0, -stheta,
	       0, 1,       0,
	  stheta, 0,  ctheta;
    Rx << 1,     0,     0,
	  0,  cphi,  sphi,
	  0, -sphi,  cphi;
    
    R = Rx * Ry * Rz;
	  
    return R;
}

Eigen::Matrix3f trans_euler_to_rotation3 ( const float& phi, const float& theta, const float& psi )
{
    float cphi = cos(phi), sphi = sin(phi);
    float ctheta = cos(theta), stheta = sin(theta);
    float cpsi = cos(psi), spsi = sin(psi);
    
    Eigen::Matrix3f Rz, Rx, Ry, R;
    Rz << cpsi, -spsi, 0,
	  spsi,  cpsi, 0,
	     0,     0, 1;
    Ry <<  ctheta, 0, stheta,
	        0, 1,      0,
	  -stheta, 0, ctheta;
    Rx << 1,     0,      0,
	  0,  cphi,  -sphi,
	  0,  sphi,   cphi;
    
    R = Rz * Ry * Rx;
    
    return R;
}

Eigen::Vector3f trans_blh_to_xyz(const Eigen::Vector3f& BLH)
{
    float B = BLH(0), L = BLH(1), H = BLH(2);
    
    float x, y, z, W, N; 
    float a = 6378137.00, b = 6356752.3142, e2 = (a*a-b*b)/(a*a);
    
    W = sqrt( 1 - e2*sin(B)*sin(B) );
    N = a / W;
    
    x = (N + H) * cos(B) * cos(L);
    y = (N + H) * cos(B) * sin(L);
    z = (N * (1-e2) + H) * sin(B);
    
    return ( Eigen::Vector3f(x, y, z) );
}
    
Eigen::Vector3f trans_blh_to_xyz(const Eigen::Vector3d& BLH, const float& scale)
{
    double lat = BLH(0), lon = BLH(1), alt = BLH(2);
    double er = 6378137.00;
    
    double x = scale * lon * er;
    double y = scale * er * log( tan( (M_PI/4) + (lat/2) ) );
    double z = alt;
    
    // std::cout << std::fixed << "x = " << x << ", y = " << y << std::endl;
     
    return ( Eigen::Vector3f(x, y, z) );
}

// Navigation system -> WGS84
Eigen::Matrix3f trans_bl_to_rotation(const float& B, const float& L)
{
    float cB = cos(B), sB = sin(B);
    float cL = cos(L), sL = sin(L);
    
    Eigen::Matrix3f R;
    // NED 
    /*R << cB * cL,  cB * sL,  -sB,
	      -sL,       cL,    0,
	  cL * sB,  sL * sB,   cB;*/
    
    // ENU
    R << -sB * cL,  -sB * sL,  cB,
	      -sL,        cL,   0,
	  cB * cB,   sL * cB,  sB;
	  
    return R;
}

Eigen::Matrix4f trans_matrix ( const Eigen::Matrix3f& R, const Eigen::Vector3f& t )
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.topLeftCorner<3, 3>() = R;
    T.topRightCorner<3, 1>() = t;
    
    return T;
}

Eigen::Vector3f trans_rotation_to_euler ( const Eigen::Matrix3f& rotation )
{
    Eigen::Vector3f euler;
    float r, p, y;
    p = -asin(rotation(2,0));
    r = atan2(rotation(1,0) / cos(p), rotation(0,0) / cos(p));
    y = atan2(rotation(2,1) / cos(p), rotation(2,2) / cos(p));
	
    euler[0] = r;
    euler[1] = p;
    euler[2] = y;
    
    return euler;
}

void depth_to_color ( cv::Mat& color, const cv::Mat& depth, const double max, const double min )
{ 
    cv::Mat grayImage; 
    double alpha = 255.0 / (max - min); 
    depth.convertTo(grayImage, CV_8UC1, alpha, -alpha * min);
    cv::applyColorMap(grayImage, color, cv::COLORMAP_HSV);
}

float to_degree ( const float& rad )
{
    float degree = rad * 180.0f / M_PI;
    return degree;
}

float to_radian ( const float& degree )
{
    float rad = degree * M_PI / 180.0f;
    return rad;
}

}