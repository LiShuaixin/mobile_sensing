#include <depth_ground_remover.h>

namespace Mobile_Sensing {

using cv::Mat;
using cv::DataType;
using std::to_string;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

cv::Mat DepthGroundRemover::zero_out_ground(const cv::Mat& image, const cv::Mat& angle_image, const float& threshold) const 
{
    // TODO(igor): test if its enough to remove only values starting from the
    // botom pixel. I don't like removing all values based on a threshold.
    // But that's a start, so let's stick with it for now.
    cv::Mat res = cv::Mat::zeros(image.size(), CV_32F);
    for (int r = 0; r < image.rows; ++r) 
    {
	for (int c = 0; c < image.cols; ++c) 
	{
	    if ( angle_image.at<float>(r, c) > to_radian(threshold) )
		res.at<float>(r, c) = image.at<float>(r, c);
	}
    }
    return res;
}

cv::Mat DepthGroundRemover::zero_out_ground_BFS(const cv::Mat& image, const cv::Mat& angle_image, const float& threshold, int kernel_size) const 
{
    cv::Mat res = cv::Mat::zeros(image.size(), CV_32F);
    depth_clustering::Radians angle_threshold(depth_clustering::Radians::IsRadians{}, to_radian(threshold) );
    depth_clustering::LinearImageLabeler<> image_labeler(image, projection_params_, angle_threshold);
    depth_clustering::SimpleDiff simple_diff_helper(&angle_image);
    float start_thresh = 30.;
    for (int c = 0; c < image.cols; ++c) 
    {
	// start at bottom pixels and do bfs
	int r = image.rows - 1;
	while (r > 0 && image.at<float>(r, c) < 0.001f)
	    --r;

	auto current_coord = depth_clustering::PixelCoord(r, c);
	uint16_t current_label = image_labeler.LabelAt(current_coord);
	if (current_label > 0)
	    // this coord was already labeled, skip
	    continue;

	// TODO(igor): this is a test. Maybe switch it on, maybe off.
	if (angle_image.at<float>(r, c) > to_radian(start_thresh) )
	    continue;

	image_labeler.LabelOneComponent(1, current_coord, &simple_diff_helper);
    }
    
    auto label_image_ptr = image_labeler.GetLabelImage();
    if (label_image_ptr->rows != res.rows || label_image_ptr->cols != res.cols) 
    {
	fprintf(stderr, "ERROR: label image and res do not correspond.\n");
	return res;
    }
    
    kernel_size = std::max(kernel_size - 2, 3);
    cv::Mat kernel = get_uniform_kernel(kernel_size, CV_8U);
    cv::Mat dilated = Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
    cv::dilate(*label_image_ptr, dilated, kernel);
    for (int r = 0; r < dilated.rows; ++r) 
    {
	for (int c = 0; c < dilated.cols; ++c) 
	{
	    if (dilated.at<uint16_t>(r, c) == 0)
		// all unlabeled points are non-ground
		res.at<float>(r, c) = image.at<float>(r, c);
	}
    }
    return res;
}

cv::Mat DepthGroundRemover::repair_depth_image(const Mat& no_ground_image, int step, float depth_threshold) 
{
    cv::Mat inpainted_depth = no_ground_image.clone();
    for (int c = 0; c < inpainted_depth.cols; ++c) {
	for (int r = 0; r < inpainted_depth.rows; ++r) {
	    float& curr_depth = inpainted_depth.at<float>(r, c);
	    if (curr_depth < 0.001f) {
		int counter = 0;
		float sum = 0.0f;
		for (int i = 1; i < step; ++i) {
		    if (r - i < 0)
			continue;

		    for (int j = 1; j < step; ++j) {
			if (r + j > inpainted_depth.rows - 1)
			    continue;

			const float& prev = inpainted_depth.at<float>(r - i, c);
			const float& next = inpainted_depth.at<float>(r + j, c);
			
			if (prev > 0.001f && next > 0.001f && fabs(prev - next) < depth_threshold) {
			    sum += prev + next;
			    counter += 2;
			}
		    }
		}
		
		if (counter > 0)
		    curr_depth = sum / counter;

	    }
	}
    }
    
    return inpainted_depth;
}

cv::Mat DepthGroundRemover::repair_depth_image (const cv::Mat& depth_image) 
{
    cv::Mat kernel = get_uniform_kernel(5);
    cv::Mat inpainted_depth;  // init an empty smoothed image
    cv::filter2D(depth_image, inpainted_depth, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER, 0, cv::BORDER_REFLECT101);
    cv::Mat mask = depth_image > 0;
    depth_image.copyTo(inpainted_depth, mask);
    return inpainted_depth;
}

cv::Mat DepthGroundRemover::create_angle_image ( const Mat& depth_image ) 
{
    cv::Mat angle_image = cv::Mat::zeros(depth_image.size(), DataType<float>::type);
    cv::Mat x_mat = cv::Mat::zeros(depth_image.size(), DataType<float>::type);
    cv::Mat y_mat = cv::Mat::zeros(depth_image.size(), DataType<float>::type);
    const auto& sines_vec = projection_params_.row_angle_sines();
    const auto& cosines_vec = projection_params_.row_angle_cosines();
    float dx, dy;
    x_mat.row(0) = depth_image.row(0) * cosines_vec[0];
    y_mat.row(0) = depth_image.row(0) * sines_vec[0];
    for (int r = 1; r < angle_image.rows; ++r) {
	x_mat.row(r) = depth_image.row(r) * cosines_vec[r];
	y_mat.row(r) = depth_image.row(r) * sines_vec[r];
	for (int c = 0; c < angle_image.cols; ++c) {
	    dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));
	    dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));
	    angle_image.at<float>(r, c) = atan2(dy, dx);
	}
    }
    return angle_image;
}

cv::Mat DepthGroundRemover::get_savitsky_golay_kernel (int window_size) const 
{
    if (window_size % 2 == 0)
	throw std::logic_error("only odd window size allowed");

    bool window_size_ok = window_size == 5 || window_size == 7 || window_size == 9 || window_size == 11;
    if (!window_size_ok)
	throw std::logic_error("bad window size");

    // below are no magic constants. See Savitsky-golay filter.
    cv::Mat kernel;
    switch (window_size) {
	case 5:
	{
	    kernel = cv::Mat::zeros(window_size, 1, CV_32F);
	    kernel.at<float>(0, 0) = -3.0f;
	    kernel.at<float>(0, 1) = 12.0f;
	    kernel.at<float>(0, 2) = 17.0f;
	    kernel.at<float>(0, 3) = 12.0f;
	    kernel.at<float>(0, 4) = -3.0f;
	    kernel /= 35.0f;
	    return kernel;
	}
	case 7:
	{
	    kernel = cv::Mat::zeros(window_size, 1, CV_32F);
	    kernel.at<float>(0, 0) = -2.0f;
	    kernel.at<float>(0, 1) = 3.0f;
	    kernel.at<float>(0, 2) = 6.0f;
	    kernel.at<float>(0, 3) = 7.0f;
	    kernel.at<float>(0, 4) = 6.0f;
	    kernel.at<float>(0, 5) = 3.0f;
	    kernel.at<float>(0, 6) = -2.0f;
	    kernel /= 21.0f;
	    return kernel;
	}
	case 9:
	{
	    kernel = cv::Mat::zeros(window_size, 1, CV_32F);
	    kernel.at<float>(0, 0) = -21.0f;
	    kernel.at<float>(0, 1) = 14.0f;
	    kernel.at<float>(0, 2) = 39.0f;
	    kernel.at<float>(0, 3) = 54.0f;
	    kernel.at<float>(0, 4) = 59.0f;
	    kernel.at<float>(0, 5) = 54.0f;
	    kernel.at<float>(0, 6) = 39.0f;
	    kernel.at<float>(0, 7) = 14.0f;
	    kernel.at<float>(0, 8) = -21.0f;
	    kernel /= 231.0f;
	    return kernel;
	}
	case 11:
	{
	    kernel = cv::Mat::zeros(window_size, 1, CV_32F);
	    kernel.at<float>(0, 0) = -36.0f;
	    kernel.at<float>(0, 1) = 9.0f;
	    kernel.at<float>(0, 2) = 44.0f;
	    kernel.at<float>(0, 3) = 69.0f;
	    kernel.at<float>(0, 4) = 84.0f;
	    kernel.at<float>(0, 5) = 89.0f;
	    kernel.at<float>(0, 6) = 84.0f;
	    kernel.at<float>(0, 7) = 69.0f;
	    kernel.at<float>(0, 8) = 44.0f;
	    kernel.at<float>(0, 9) = 9.0f;
	    kernel.at<float>(0, 10) = -36.0f;
	    kernel /= 429.0f;
	    return kernel;
	}
    }
    return kernel;
}

cv::Mat DepthGroundRemover::get_uniform_kernel (int window_size, int type) const 
{
    if (window_size % 2 == 0)
	throw std::logic_error("only odd window size allowed");

    cv::Mat kernel = cv::Mat::zeros(window_size, 1, type);
    kernel.at<float>(0, 0) = 1;
    kernel.at<float>(window_size - 1, 0) = 1;
    kernel /= 2;
    return kernel;
}

cv::Mat DepthGroundRemover::apply_savitsky_golay_smoothing (const cv::Mat& image, int window_size) 
{
    cv::Mat kernel = get_savitsky_golay_kernel(window_size);

    cv::Mat smoothed_image;  // init an empty smoothed image
    cv::filter2D(image, smoothed_image, SAME_OUTPUT_TYPE, kernel, ANCHOR_CENTER, 0, cv::BORDER_REFLECT101);
    return smoothed_image;
}

float DepthGroundRemover::get_line_angle(const cv::Mat& depth_image, int col, int row_curr, int row_neigh) 
{
    // compute inclination angle of the line given the depth of two pixels and
    // their position in the image. We use config to determine the needed angles
    // All following angles are in degrees
    float current_angle;
    float neighbor_angle;
    current_angle = projection_params_.get_angle_from_row(row_curr);
    neighbor_angle = projection_params_.get_angle_from_row(row_neigh);
    
    // for easiness copy references to depth of current and neighbor positions
    const float& depth_current = depth_image.at<float>(row_curr, col);
    const float& depth_neighbor = depth_image.at<float>(row_neigh, col);
    if (depth_current < eps_ || depth_neighbor < eps_)
	// if either of these depth vales is close to zero this depth is not
	// reliable, so we will just report a 0 instead.
	return 0.;

    auto x_current = depth_current * cos( to_radian(current_angle) );
    auto y_current = depth_current * sin( to_radian(current_angle) );
    auto x_neighbor = depth_neighbor * cos( to_radian(neighbor_angle) );
    auto y_neighbor = depth_neighbor * sin( to_radian(neighbor_angle) );
    auto dx = fabs(x_current - x_neighbor);
    auto dy = fabs(y_current - y_neighbor);
    auto angle = std::atan2(dy, dx) * 180. / M_PI;
    return angle;
}

void DepthGroundRemover::process( Scan& scan ) 
{
    // this can be done even faster if we switch to column-major implementation
    // thus allowing us to load whole row in L1 cache
    Scan scan_copy(scan);
    const cv::Mat& depth_image = repair_depth_image(scan.scan_projection_->depth_image(), 5, 1.0f);
    
    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    auto angle_image = create_angle_image(depth_image);
    angle_image.copyTo(angle_image_);
    
    auto smoothed_image = apply_savitsky_golay_smoothing(angle_image, window_size_);
    smoothed_image.copyTo(smoothed_angle_image_);
    
    auto no_ground_image = zero_out_ground_BFS(depth_image, smoothed_image, ground_remove_angle_, window_size_);
    
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    // fprintf(stderr, "INFO: Ground removed in %lu s\n", time_used.count());
    
    scan_copy.scan_projection_->depth_image() = no_ground_image;
    
    // update pointcloud -> remove ground points in the pointcloud
    for (int row = 0; row < no_ground_image.rows; ++row) 
    {
	for (int col = 0; col < no_ground_image.cols; ++col) 
	{
	    const auto& point_container = scan_copy.scan_projection_->at(row, col);
	    if (point_container.is_empty()) // this is ok, just continue, nothing interesting here, no points.
		continue;

	    uint16_t label = no_ground_image.at<uint16_t>(row, col);
	    if (label > 0.01) // this is a default label meaning that it's not a ground pixel, skip
		continue;

	    for (const auto& point_idx : point_container.points()) 
		scan_copy.mask_.push_back(point_idx);
	}
    }
    scan_copy.mask_.sort();
    
    scan = Scan(scan_copy);
}

}  // namespace Mobile_Sensing
