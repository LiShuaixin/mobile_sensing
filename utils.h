#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <string>
#include <stdio.h>  
#include <stdlib.h> 
#include <utility>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

namespace Mobile_Sensing
{

    using std::unique_ptr;
    
    long long int parse_kitti_stamp ( const std::string& s );

    long long parse_stamp(const std::string& s);

    Eigen::Matrix3f trans_euler_to_rotation( const float& phi, const float& theta, const float& psi );

    Eigen::Matrix3f trans_euler_to_rotation2 ( const float& phi, const float& theta, const float& psi );

    Eigen::Matrix3f trans_euler_to_rotation3 ( const float& phi, const float& theta, const float& psi );
    
    Eigen::Vector3f trans_rotation_to_euler( const Eigen::Matrix3f& R );

    Eigen::Vector3f trans_blh_to_xyz(const Eigen::Vector3f& BLH);
    Eigen::Vector3f trans_blh_to_xyz(const Eigen::Vector3d& BLH, const float& scale);

    // Navigation system -> WGS84
    Eigen::Matrix3f trans_bl_to_rotation(const float& B, const float& L);

    Eigen::Matrix4f trans_matrix(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);
    
    void depth_to_color(cv::Mat& color, const cv::Mat& depth, const double max, const double min);
    
    float to_degree( const float& rad );
    
    float to_radian( const float& degree );

    
    template <typename PointT>
    float calc_squared_diff(const PointT& a, const PointT& b)
    {
	float diffX = a.x - b.x;
	float diffY = a.y - b.y;
	float diffZ = a.z - b.z;

	return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }

    template <typename PointT>
    float calc_squared_diff(const PointT& a, const PointT& b, const float& wb)
    {
	float diffX = a.x - b.x * wb;
	float diffY = a.y - b.y * wb;
	float diffZ = a.z - b.z * wb;

	return diffX * diffX + diffY * diffY + diffZ * diffZ;
    }
	
    template <typename PointT>
    float calc_point_distance(const PointT& p)
    {
	return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    template <typename PointT>
    float calc_squared_point_distance(const PointT& p)
    {
	return p.x * p.x + p.y * p.y + p.z * p.z;
    }
    
    // make shared
    template <typename T, typename... Args>
    unique_ptr<T> make_unique(Args&&... args) 
    {
	return unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

}
#endif //UTILS_H