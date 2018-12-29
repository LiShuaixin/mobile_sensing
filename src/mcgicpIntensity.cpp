#include <mcgicpIntensity.h>

namespace pcl 
{
	/** \brief constructor
	 * \param intensity_weight the scale value of the intensity
	 */
	GeneralizedIterativeClosestPoint4D::GeneralizedIterativeClosestPoint4D(float intensity_weight, float intensity_covariance) :
    cloud_intensity_(new pcl::PointCloud<PointXYZI>), target_intensity_(new pcl::PointCloud<PointXYZI>), intensity_weight_(intensity_weight), intensity_covariance_(intensity_covariance)
  {
		// set rescale mask (leave x,y,z unchanged, scale intensity by intensity_weight)
		float alpha[4] = { 1.0, 1.0, 1.0, intensity_weight_ };
	  point_rep_.setRescaleValues(alpha);
	}
	
	void GeneralizedIterativeClosestPoint4D::setInputSource(const PointCloudSourceConstPtr& cloud)
	{
		// call corresponding base class method
		GeneralizedIterativeClosestPoint<PointSource, PointTarget>::setInputSource(cloud);
		*cloud_intensity_ = *cloud;
	}
	
	void GeneralizedIterativeClosestPoint4D::setInputTarget(const PointCloudTargetConstPtr& target)
	{
		// call corresponding base class method
		GeneralizedIterativeClosestPoint<PointSource, PointTarget>::setInputTarget(target);
		
		*target_intensity_ = *target;
		
		// in addition, build 6d-tree
		target_tree_intensity_.setInputCloud(target_intensity_);
		target_tree_intensity_.setPointRepresentation(
			boost::make_shared<XYZIPointRepresentation>(point_rep_));
	}
	
	bool GeneralizedIterativeClosestPoint4D::searchForNeighbors(const pcl::PointXYZI& query, std::vector<int>& index, std::vector<float>& distance)
	{
		int k = target_tree_intensity_.nearestKSearch(query, 1, index, distance);
		
		// check if neighbor was found
		return (k != 0);
	}
	
	void GeneralizedIterativeClosestPoint4D::computeCovariances(PointCloudSource::ConstPtr cloud, const pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree, MatricesVector& cloud_covariances)
	{
		// std::cout << "  covariance is rcomputing..." << std::endl;
		if (k_correspondences_ > int (cloud->size ()))
		{
			PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::computeCovariances] Number or points in cloud (%lu) is less than k_correspondences_ (%lu)!\n", cloud->size (), k_correspondences_);
			return;
		}
		
		Eigen::Vector3d mean;
		std::vector<int> nn_indecies; nn_indecies.reserve (k_correspondences_);
		std::vector<float> nn_dist_sq; nn_dist_sq.reserve (k_correspondences_);
		
		// We should never get there but who knows
		if(cloud_covariances.size () < cloud->size ())
			cloud_covariances.resize (cloud->size ());
		
		PointCloudSource::const_iterator points_iterator = cloud->begin ();
		MatricesVector::iterator matrices_iterator = cloud_covariances.begin ();
		
		// Compute every points' covariance
		for(; points_iterator != cloud->end(); ++points_iterator, ++matrices_iterator)
		{
			const pcl::PointXYZI &query_point = *points_iterator;
			Eigen::Matrix3d &cov = *matrices_iterator;
			// Zero out the cov and mean
			cov.setZero ();
			mean.setZero ();
			
			// Search for the K nearest neighbours
      kdtree->nearestKSearch(query_point, k_correspondences_, nn_indecies, nn_dist_sq);
			
			// Find the covariance matrix
			for(int j = 0; j < k_correspondences_; j++) {
				const pcl::PointXYZI &pt = (*cloud)[nn_indecies[j]];
				
				mean[0] += pt.x;
				mean[1] += pt.y;
				mean[2] += pt.z;
			}
			mean /= static_cast<double>(k_correspondences_);
			
			for(int j = 0; j < k_correspondences_; j++) {
				const pcl::PointXYZI &pt = (*cloud)[nn_indecies[j]];
				
				float ax = pt.x - mean[0];
				float ay = pt.y - mean[1];
				float az = pt.z - mean[2];
				
				cov(0,0) += ax * ax;
				cov(1,0) += ax * ay;
				cov(2,0) += ax * az;
				cov(1,1) += ay * ay;
				cov(2,1) += ay * az;
				cov(2,2) += az * az;
			}
			cov /= static_cast<double>(k_correspondences_);
			
			// Get the actual covariance
			for (int k = 0; k < 3; k++)
				for (int l = 0; l <= k; l++){
					cov(l,k) = cov(k,l);}
				
			// Compute the SVD (covariance matrix is symmetric so U = V')
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
			cov.setZero ();
			Eigen::Matrix3d U = svd.matrixU ();
			// Compute 2×3 projection matrix according to U
			Eigen::MatrixXd proj;
			proj.setZero(2, 3);
			proj.row(0) = (U.col(0)).transpose();
			proj.row(1) = (U.col(1)).transpose(); 
			
			// Find muW, muP
			Eigen::Vector2d muW,muP;			
			double lambda = 0;
			muW.setZero(); 
			muP.setZero();
			
			for(int j = 0; j < k_correspondences_; j++) {
				const pcl::PointXYZI &pt = (*cloud)[nn_indecies[j]];				
				Eigen::Vector3d lp(pt.x, pt.y, pt.z);
				Eigen::Vector2d zp = proj * lp;
				float zd = pt.intensity;	
				// std::cout << zd << std::endl;
				float qd = query_point.intensity;
				
				// Compute the weight of a selected point->lambda
				double tmp = (zd-qd) * (1/intensity_covariance_) * (zd - qd);
				double lambdai = exp( -0.5 * tmp );
				
				muW += zp;
				muP += lambdai * zp;
				lambda += lambdai;
			}
			muP /= lambda;
			muW /= static_cast<double>(k_correspondences_);
			lambda = 0;
			
			Eigen::Matrix2d sigmaW,sigmaD;
			sigmaW.setZero(); 
			sigmaD.setZero();
			for(int j = 0; j < k_correspondences_; j++) {
				const pcl::PointXYZI &pt = (*cloud)[nn_indecies[j]];
				Eigen::Vector3d lp(pt.x, pt.y, pt.z);
				Eigen::Vector2d zp = proj * lp;
				float zd = pt.intensity;				
				float qd = query_point.intensity;
				
				// Compute the weight of a selected point->lambda
				double tmp = (zd-qd) * (1/intensity_covariance_) * (zd - qd);
				double lambdai = exp( -0.5 * tmp );
				
				sigmaW += (zp - muW) * (zp - muW).transpose();
				sigmaD += lambdai * (zp - muP) * (zp - muP).transpose();
				lambda += lambdai;
			}
			sigmaD /= lambda;
			sigmaW /= static_cast<double>(k_correspondences_);
			
			// Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
			Eigen::Matrix2d Omega;
			Omega(0, 0) = (1/sqrt(sigmaW(0,0))) * sigmaD(0,0) * (1/sqrt(sigmaW(0,0)));
			Omega(1, 1) = (1/sqrt(sigmaW(1,1))) * sigmaD(1,1) * (1/sqrt(sigmaW(1,1)));
			
			Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
			C.topLeftCorner<2, 2>() = Omega;
			C(2,2) = gicp_epsilon_;
			
			cov = U * C * U.transpose();
		}
	}

	// take from the original GICP class and modified slightly to make use of intensity values
	void GeneralizedIterativeClosestPoint4D::computeTransformation(PointCloudSource& output, const Eigen::Matrix4f& guess)
	{
		// std::cout << "MCGICP is runing..." << std::endl;

		using namespace pcl;
		using namespace std;
		
		// Initialize
		IterativeClosestPoint<PointSource, PointTarget>::initComputeReciprocal();
		
		// Difference between consective transforms
		double delta = 0;
		// Get the size of the target
		const size_t N = indices_->size();
		
		// Set the mahalanobis matrix to identity
		mahalanobis_.resize(N, Eigen::Matrix3d::Identity());
		
		// Compute target cloud covariance matrix
		if((!target_covariances_) || (target_covariances_->empty()))
		{
			target_covariances_.reset(new MatricesVector);
			computeCovariances(target_, tree_, *target_covariances_);
		}
		
		// compute input cloud covariance matrix
		if((!input_covariances_) || (input_covariances_->empty()))
		{
			input_covariances_.reset(new MatricesVector);
			computeCovariances(input_, tree_reciprocal_, *input_covariances_);
		}
		
		base_transformation_ = guess;
		nr_iterations_ = 0;
		converged_ = false;
		double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
		vector<int> nn_indices(1);
		vector<float> nn_dists(1);
		
		/*output = *cloud_intensity_;
		pcl::transformPointCloud(output, output, guess);*/
		
		while(!converged_)
		{
			size_t cnt = 0;
			vector<int> source_indices(indices_->size());
			vector<int> target_indices(indices_->size());
			
			// guess corresponds to base_t and tranformation_ to t
			Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero();
			for(size_t i = 0; i<4; i++)
				for(size_t j = 0; j<4; j++)
					for(size_t k = 0; k<4; k++)
						transform_R(i, j) += double(transformation_(i, k)) * double(guess(k, j));
					
		  Eigen::Matrix3d R = transform_R.topLeftCorner<3, 3>();
			
			for(size_t i = 0; i<N; i++)
			{
				// Modification:: take point from the intensity cloud instead
				PointXYZI query = (*cloud_intensity_)[i];
				query.getVector4fMap() = guess * query.getVector4fMap();
				query.getVector4fMap() = transformation_ * query.getVector4fMap();
				
				if(!searchForNeighbors(query, nn_indices, nn_dists))
				{
					PCL_ERROR(
						"[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n",
						getClassName ().c_str (), (*indices_)[i]);
					return;
				}
				
				// Check if the distance to the nearest neighbor is smaller than the user imposed threshold
				if(nn_dists[0] < dist_threshold)
				{
					Eigen::Matrix3d &C1 = (*input_covariances_)[i];
					Eigen::Matrix3d &C2 = (*target_covariances_)[nn_indices[0]];
					Eigen::Matrix3d &M = mahalanobis_[i];
					
					// M = R * C1;
					M = R * C1;
					// temp = M*R' + C2 = R*C1*R' + C2
					Eigen::Matrix3d temp = M * R.transpose();
					temp += C2;
					// M = temp^ -1 
					M = temp.inverse();
					source_indices[cnt] = static_cast<int> (i);
					target_indices[cnt] = nn_indices[0];
					cnt++;
				}
			}
			
			// Resize to the actual number of valid correspondences
			source_indices.resize(cnt);
			target_indices.resize(cnt);
			// Optimize transformation using the current assignment and Mahalanobis matrix
			previous_transformation_ = transformation_;
			
			// Optimization right here
			try
			{
				rigid_transformation_estimation_(output, source_indices, *target_, target_indices, transformation_);
				// compute the delta from this iteration
				delta = 0.;
				// Find the maximum difference between element of the previous and the current transformation.
				for(int k = 0; k<4; k++)
				{
					for(int l = 0; l<4; l++)
					{
						double ratio = 1;
						if(k<3 && l<3) // rotationpart of the transform
							ratio = 1. / rotation_epsilon_;
						else
							ratio = 1. / transformation_epsilon_;
						double c_delta = ratio * fabs(previous_transformation_(k,l) - transformation_(k,l));
						if(c_delta > delta)
							delta = c_delta;
					}
				}
			}
			catch(PCLException &e)
			{
				PCL_DEBUG("[pcl::%s::computeTransformation] Optimization issue %s\n", getClassName ().c_str (), e.what ());
				break;
			}
			
			nr_iterations_++;
			// check for convergence
			if(nr_iterations_ >= max_iterations_ || delta < 1)
			{
				converged_ = true;
				previous_transformation_ = transformation_;
				PCL_DEBUG(
					"[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
					getClassName ().c_str (), nr_iterations_, max_iterations_,
					(transformation_ - previous_transformation_).array ().abs ().sum ());
			}
			else
				PCL_DEBUG("[pcl::%s::computeTransformation] Convergence failed\n",
					getClassName ().c_str ());
		}
		
		// for some reason the static equivalent method raises an error
		final_transformation_.topLeftCorner(3, 3) = previous_transformation_.topLeftCorner(3, 3) * guess.topLeftCorner(3, 3);
		final_transformation_(0, 3) = previous_transformation_(0, 3) + guess(0, 3);
		final_transformation_(1, 3) = previous_transformation_(1, 3) + guess(1, 3);
		final_transformation_(2, 3) = previous_transformation_(2, 3) + guess(2, 3);
		
		// Transform the point cloud
		pcl::transformPointCloud(*input_, output, final_transformation_);
	}
}
