#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <iostream>
#include <list>
#include <time.h>
#include <chrono>

#include <tbb/tbb.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <pcl/common/time.h>
#include <pcl/registration/icp.h>           
#include <pcl/registration/icp_nl.h>        
#include <pcl/registration/gicp.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>    

#include <scan.h>
#include <mcgicpIntensity.h>
#include <surfel_map/map_point_types.h>
#include <surfel_map/abstract_surfel_map.h>

namespace Mobile_Sensing
{
struct RegistrationParameters
{
    RegistrationParameters()
    : associate_once_(true)
    , prior_prob_(0.9)
    , sigma_size_factor_(0.25)
    , soft_assoc_c1_(1.0)
    , soft_assoc_c2_(8.0)
    , soft_assoc_c3_(1.0)
    , max_iterations_(100)
    , min_iterations_(15)
    , is_downsize_(true)
    , downsize_scale_(0.4)
    , intensity_variance_(0.05)
    , r_variance_(0.1)
    , g_variance_(0.1)
    , b_variance_(0.1)
    , convergence_threshold_(1e-4)
    , max_correspondence_dist_(0.5)
    {}

    bool associate_once_;
    
    double prior_prob_;

    double sigma_size_factor_;

    double soft_assoc_c1_, soft_assoc_c2_, soft_assoc_c3_;
  
    int max_iterations_;
    int min_iterations_;
    
    bool is_downsize_;
    
    float downsize_scale_;
    
    float intensity_variance_;
    
    float r_variance_;
    float g_variance_;
    float b_variance_;
    
    float convergence_threshold_;
    float max_correspondence_dist_;
};

class Registration
{
public: 
  
    Registration();
    
    Registration(const Registration& registrator);
    
    Registration(const RegistrationParameters& params);
    
    ~Registration(){}

    class SingleAssociation
    {
    public:
	SingleAssociation() : cell_scene_(NULL), cell_model_(NULL), match(0) {}
	SingleAssociation(Mobile_Sensing::AbstractSurfelCell* cell_scene, Mobile_Sensing::AbstractSurfelCell* cell_model)
	: cell_scene_(cell_scene), cell_model_(cell_model), match(1) {}
	~SingleAssociation() {}

	Mobile_Sensing::AbstractSurfelCell* cell_scene_;
	Mobile_Sensing::AbstractSurfelCell* cell_model_;

	double error;
	double weight;
	double level;
	double sigma, inv_sigma2;
	int match;

	Eigen::Vector3d model_mean;
	Eigen::Vector3d scene_mean;

	// for Levenberg-Marquardt
	// (z - f)^T W (z - f)
	Eigen::Vector3d z, f;  //, df_qx, df_qy, df_qz;
	Eigen::Matrix<double, 3, 6> df_dx;
	Eigen::Matrix3d W;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    typedef std::vector<SingleAssociation, Eigen::aligned_allocator<SingleAssociation>> SingleAssociationList;

    class SceneSurfelAssociation
    {
    public:
	SceneSurfelAssociation() : cell_scene_(NULL) {}
	~SceneSurfelAssociation() {}

	Mobile_Sensing::AbstractSurfelCell* cell_scene_;

	unsigned int model_points_;

	SingleAssociationList associations_;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    typedef std::vector<SceneSurfelAssociation, Eigen::aligned_allocator<SceneSurfelAssociation>> SceneSurfelAssociationList;

    class CellInfo
    {
    public:
	Mobile_Sensing::AbstractSurfelCell* cell_;
	Eigen::Vector3d offset_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    typedef std::vector<CellInfo, Eigen::aligned_allocator<CellInfo>> CellInfoList;

    struct RegistrationFunctionParameters
    {
	Mobile_Sensing::AbstractSurfelMap* model;
	Mobile_Sensing::AbstractSurfelMap* scene;
	Registration::CellInfoList scene_cells;

	unsigned int model_num_points_, scene_num_points_;

	double prior_prob;
	double sigma_size_factor;

	double soft_assoc_c1, soft_assoc_c2, soft_assoc_c3;

	Eigen::Matrix4d* transform;
	float lastWSign;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondences_source_points_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondences_target_points_;
    };

    void set_registration_parameters(const RegistrationParameters& params);

    void associate_maps_breadth_first_parallel(SceneSurfelAssociationList& surfelAssociations, Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene,
                                         Registration::CellInfoList& sceneCells,
                                         Eigen::Matrix4d& transform, double sigma);

    bool estimate_transformation_levenbergmarquardt(Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene, Eigen::Matrix4d& transform,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints,
                                                  int maxIterations);

    bool estimate_transformation_levenbergmarquardt(Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene, Eigen::Matrix4d& transform,
                                                int maxIterations)
    {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints;
	return estimate_transformation_levenbergmarquardt(model, scene, transform, correspondencesSourcePoints,
							correspondencesTargetPoints, maxIterations);
    }

    bool estimate_transformation_levenbergmarquardt(Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene, Eigen::Matrix4d& transform,
						    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
						    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints)
    {
	return estimate_transformation_levenbergmarquardt(model, scene, transform, correspondencesSourcePoints,
							correspondencesTargetPoints, max_iterations_);
    }
  
    bool estimate_transformation_levenbergmarquardt(Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene, Eigen::Matrix4d& transform)
    {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints;
	return estimate_transformation_levenbergmarquardt(model, scene, transform, correspondencesSourcePoints,
							correspondencesTargetPoints, max_iterations_);
    }
  
    bool estimate_pose_covariance(Eigen::Matrix<double, 6, 6>& poseCov, Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene,
                              Eigen::Matrix4d& transform);
    bool estimate_pose_covariance_unscented(Eigen::Matrix<double, 6, 6>& poseCov, Mobile_Sensing::AbstractSurfelMap* model, Mobile_Sensing::AbstractSurfelMap* scene,
                                       Eigen::Matrix4d& transform);

    void set_prior_pose_enabled(bool enabled) { use_prior_pose_ = enabled; }
    void set_prior_pose(bool enabled, const Eigen::Matrix<double, 6, 1>& prior_pose_mean,
                    const Eigen::Matrix<double, 6, 1>& prior_pose_variances);

    Eigen::Matrix<double, 6, 1> transform2pose(const Eigen::Matrix4d& transform, double& qw_sign);
    Eigen::Matrix4d pose2transform(const Eigen::Matrix<double, 6, 1>& pose, double qw_sign);

    double mcgicp_registration(Scan& src, Scan& tgt, Eigen::Matrix4f& transform);
    double mcgicp_registration(PointCloud::Ptr src, PointCloud::Ptr tgt, Eigen::Matrix4f& transform);
    double mcgicp_registration(AbstractSurfelMap* model, AbstractSurfelMap* scene, Eigen::Matrix4f& transform);
    double mcgicp_registration(AbstractSurfelMap* model, PointCloud::Ptr scene, Eigen::Matrix4f& transform);
    
    double two_step_registration(Scan& src, Scan& tgt, Eigen::Matrix4f& transform);

protected:
    bool registration_error_function_LM(const Eigen::Matrix<double, 6, 1>& x,
				    Registration::RegistrationFunctionParameters& params, double& f,
				    Registration::SceneSurfelAssociationList& associations);
    bool registration_error_function_with_first_and_second_derivative_LM(
	const Eigen::Matrix<double, 6, 1>& x, Registration::RegistrationFunctionParameters& params,
	double& f, Eigen::Matrix<double, 6, 1>& df, Eigen::Matrix<double, 6, 6>& d2f,
	Registration::SceneSurfelAssociationList& associations);

    // exposed parameters
    bool use_prior_pose_;
    Eigen::Matrix<double, 6, 1> prior_pose_mean_;
    Eigen::Matrix<double, 6, 6> prior_pose_invcov_;

    Eigen::Matrix<double, 6, 6> last_cov_;

    double prior_prob_;

    double sigma_size_factor_;

    double soft_assoc_c1_, soft_assoc_c2_, soft_assoc_c3_;

    bool associate_once_;
    
    int max_iterations_;
    int min_iterations_;
    // 		tbb::task_scheduler_init init_;
    
    float convergence_threshold_;
    
    float max_correspondence_dist_;
    
    bool is_downsize_;
    float downsize_scale_;
    
    float intensity_variance_;   
    float r_variance_;
    float g_variance_;
    float b_variance_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};



}


#endif