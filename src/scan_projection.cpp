#include <scan_projection.h>

namespace Mobile_Sensing 
{
    
ScanProjection::ScanProjection()
{
    projection_params_ = new ProjectionParams();
    data_ = PointMatrix( projection_params_->cols(), PointColumn(projection_params_->rows()) );
    depth_image_ = cv::Mat::zeros( projection_params_->rows(), projection_params_->cols(), cv::DataType<float>::type );
}

ScanProjection::ScanProjection ( const ScanProjection& scan_proj ) : 
                projection_params_(scan_proj.projection_params()),
                data_(scan_proj.matrix()),
                corrections_(scan_proj.corrections()),
                depth_image_(scan_proj.depth_image()) {}

ScanProjection::ScanProjection ( ProjectionParams* params ) : projection_params_(params)
{
    if (!projection_params_->valid())
	throw std::runtime_error("_params not valid for projection.");
    
    data_ = PointMatrix( projection_params_->cols(), PointColumn(projection_params_->rows()) );
    depth_image_ = cv::Mat::zeros( projection_params_->rows(), projection_params_->cols(), cv::DataType<float>::type);
}

pcl::PointXYZI ScanProjection::unproject_points ( const cv::Mat& image, const int row, const int col ) const
{
    float depth = image.at<float>(row, col);
    float angle_z = this->projection_params_->get_angle_from_row(row);
    float angle_xy = this->projection_params_->get_angle_from_col(col);
    
    pcl::PointXYZI point;
    point.x = depth * cos(angle_z) * cos(angle_xy);
    point.y = depth * cos(angle_z) * sin(angle_xy);
    point.z = depth * sin(angle_z);

    return point;
}

void ScanProjection::check_scan ( pcl::PointCloud< pcl::PointXYZI >::Ptr points )
{
    if (this->data_.size() < 1) 
	throw std::length_error("_data size is < 1");
  
    if (points->empty())
        throw std::runtime_error("cannot fill from cloud: no points"); 
}

void ScanProjection::check_image ( const cv::Mat& image )
{
    if (image.type() != CV_32F)
	throw std::runtime_error("wrong image format");
    
    if (this->data_.size() < 1) 
	throw std::length_error("_data size is < 1");
    
    if (this->rows() != static_cast<size_t>(image.rows) || this->cols() != static_cast<size_t>(image.cols))
	throw std::length_error("_data dimentions do not correspond to image ones");
}

void ScanProjection::fix_depth_systematic_error()
{
    if (depth_image_.rows < 1) 
    {
	std::fprintf(stderr, "[INFO]: image of wrong size, not correcting depth\n");
        return;
    }
    
    if (corrections_.size() != static_cast<size_t>(depth_image_.rows)) 
    {
	std::fprintf(stderr, "[INFO]: Not correcting depth data.\n");
	return;
    }
    
    for (int r = 0; r < depth_image_.rows; ++r) 
    {
	auto correction = corrections_[r];
	for (int c = 0; c < depth_image_.cols; ++c) 
	{
	    if (depth_image_.at<float>(r, c) < 0.001f) 
		continue;
	    
	    depth_image_.at<float>(r, c) -= correction;
	}
    }
}

const cv::Mat& ScanProjection::depth_image() const
{
    return this->depth_image_;
}

cv::Mat& ScanProjection::depth_image()
{
    return this->depth_image_;
}

void ScanProjection::init_from_points ( pcl::PointCloud< pcl::PointXYZI >::Ptr points )
{
    this->check_scan(points);
    for (size_t index = 0; index < points->size(); ++index) 
    {
        pcl::PointXYZI point = points->points[index];
        float dist_to_sensor = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (dist_to_sensor < 0.01f)
	    continue;
	
	auto angle_rows = (asin(point.z / dist_to_sensor)) * 180. / M_PI;
	auto angle_cols = (atan2(point.y, point.x)) * 180. / M_PI;
	size_t bin_rows = this->projection_params()->get_row_from_angle(angle_rows);
	size_t bin_cols = this->projection_params()->get_col_from_angle(angle_cols);
	// adding point pointer
	this->at(bin_rows, bin_cols).points().push_back(index); // 每一个pointcontainer中包含了点云深度影像某一像素所对应的全部点的索引
	
	auto& current_written_depth = this->depth_image_.template at<float>(bin_rows, bin_cols);
	if (current_written_depth < dist_to_sensor){
	    // write this point as the longest point which hit on this pixel
	    current_written_depth = dist_to_sensor;
	    // printf("current_written_depth = %f at [%d, %d].\n", current_written_depth, bin_rows, bin_cols);
	}
    }

    fix_depth_systematic_error();
}

ScanProjection::PointContainer::PointContainer() {}

}