#include <spanpose.h>
#include <thread>

namespace Mobile_Sensing
{

SPANPose::SPANPose() : roll_ (0.0), pitch_(0.0), heading_(0.0), timestamp_(0), twc_(Eigen::Vector3f::Zero()), Rwc_(Eigen::Matrix3f::Identity()) { }
  
SPANPose::SPANPose ( const SPANPose& pose ) : roll_ (pose.roll_), pitch_(pose.pitch_), heading_(pose.heading_), twc_(pose.twc_), Rwc_(pose.Rwc_), timestamp_(pose.timestamp_) {}

void SPANPose::set ( SPANPose& pose )
{
    roll_ = pose.roll_;
    pitch_ = pose.pitch_;
    heading_ = pose.heading_;
    twc_ = pose.twc_;
    Rwc_ = pose.Rwc_;
    timestamp_ = pose.timestamp_;
}

}
