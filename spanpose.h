#ifndef SPANPOSE_H
#define SPANPOSE_H

#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <boost/make_shared.hpp>

#include <utils.h>

namespace Mobile_Sensing
{

class SPANPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    SPANPose ();
    
    SPANPose (const SPANPose& pose);
    
    void set(SPANPose& pose);
    
    // ENU
    float roll_, pitch_, heading_;
    
    Eigen::Vector3f twc_; // IMU center
    Eigen::Matrix3f Rwc_; 

    long long timestamp_;
};

}

#endif