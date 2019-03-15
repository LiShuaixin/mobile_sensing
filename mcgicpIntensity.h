#ifndef MCGICP_H
#define MCGICP_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/gicp.h>

namespace pcl
{
	/** \brief GneralizedIterativeClosestPoint4D integrates intensity space information into the
	 * Generalized Iterative Closest Point (GICP) algorithm
	 * 
	 * The suggested input is PointXYZI.
	 * \note This code is the part of improved LOAM
	 * if you use this code in any academic work, please cite:
	 * 
	 * 
	 * \author Shuaixin Li
	 */
	class PCL_EXPORTS GeneralizedIterativeClosestPoint4D : public GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI>
	{
		typedef PointXYZI PointSource;
		typedef PointXYZI PointTarget;
		
	public:
		/** \brief constructor
		 * 
		 * \param[in] intensity_weight the intensity weight
		 */
		GeneralizedIterativeClosestPoint4D(float intensity_weight = 0.05f, float intensity_covariance = 200.f);
		
		/** \brief Provide a pointer to the input source
		 * (e.g., the point cloud that we want to align to the target)
		 * 
		 * \param[in] cloud the input point cloud source
		 */
		void setInputSource(const PointCloudSourceConstPtr& cloud);
		
		/** \brief Provide a pointer to the input target
		 * (e.g., the point cloud that we want to align the input source to)
		 * 
		 * \param[in] cloud the input point cloud target
		 */
		void setInputTarget(const PointCloudTargetConstPtr& cloud);
		
	protected:
		/** \brief Compute covariance of point cloud
		 * \param[in] cloud the point cloud for computing
		 * \param[in] kdtree kdtree for point research
		 * \param[out] cloud_covariances covariance of every points
		 */
		void computeCovariances(PointCloudSource::ConstPtr cloud, const pcl::search::KdTree<pcl::PointXYZI>::Ptr tree, MatricesVector& cloud_covariances);
		
		/** \brief Rigid transformation computation method with initial guess
		 * \param output the transformed input point cloud dataset using the rigid transformation found
		 * \param guess the initial guess of the transformation to compute
		 */
		void computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess);
		
		/** \brief Search for the closest nearest neighbor of a given point.
		 * \param query the point to search a nearest neighbor for
		 * \param index vector of size 1 to store the index of the nearest neighbor found
		 * \param distance vector of size 1 to store the distance to nearest neighbor found
		 */
		inline bool searchForNeighbors(const pcl::PointXYZI& query, std::vector<int>& index, std::vector<float>& distance);
		
	protected:
		/** \brief Holds the converted (intensity) data_cloud. 
		 */
		pcl::PointCloud<PointXYZI>::Ptr cloud_intensity_;
		
		/** \brief Holds the converted (intensity) model cloud. 
		 */
		pcl::PointCloud<PointXYZI>::Ptr target_intensity_;
		
		/** \brief 4d-tree to search in model cloud. 
		 */
		KdTreeFLANN<PointXYZI> target_tree_intensity_;
		
		/** \brief The color weight. 
		 */
		float intensity_weight_;
		
		/** \brief intensity measurement covariance
		 */
		float intensity_covariance_;
		
		/**  \brief Custom point representation to perform kdtree 
		 * searches in more than 3 (i.e. in all 4) dimensions. 
		 */
		class XYZIPointRepresentation : public PointRepresentation<pcl::PointXYZI>
		{
			using PointRepresentation<PointXYZI>::nr_dimensions_;
			using PointRepresentation<PointXYZI>::trivial_;
			
		public:
			typedef boost::shared_ptr<XYZIPointRepresentation> Ptr;
			typedef boost::shared_ptr<const XYZIPointRepresentation> ConstPtr;
			
			XYZIPointRepresentation()
			{
				nr_dimensions_ = 4;
				trivial_ = false;
			}
			
			virtual
			~XYZIPointRepresentation()
			{
			}
			
			inline Ptr makeShared() const
			{
				return Ptr (new XYZIPointRepresentation(*this));
			}
			
			virtual void copyToFloatArray(const PointXYZI &p, float *out) const
			{
				// copy all of the four values
				out[0] = p.x;
				out[1] = p.y;
				out[2] = p.z;
				out[3] = p.intensity;
			}
		};
		
		/** \brief Enables 4d searches with kd-tree class using 
		 * the intensity weight. 
		 */
		XYZIPointRepresentation point_rep_;
	};
}

#endif 