#ifndef FRAME_DRAWER_H
#define FRAME_DRAWER_H

#include <scan.h>
#include <mapping.h>
#include <projection_params.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

typedef unsigned short ushort;
typedef unsigned char uchar;

namespace Mobile_Sensing
{

class Mapping;
class ProjectionParams;

class Depth
{
public:

    /// constructor
    Depth() : depth_data_(nullptr), row_(0), col_(0) {}
    Depth(const Depth& depth) : depth_data_(depth.depth_data_), row_(depth.row_), col_(depth.col_) {}
    Depth(const ushort* depth_data, const int& row, const int& col) : row_(row), col_(col) 
    {
	try
	{
	    if (depth_data == nullptr)
		    throw("parameter of depth constructor is null\n");
	}
	catch (const std::string& e)
	{
	    std::cout << e << "in: " << __FILE__ << "line: " << __LINE__ << std::endl;
	}
	depth_data_ = new ushort[row_ * col_];
        std::copy(depth_data, depth_data + row * col, depth_data_);
    }
    
    ~Depth() 
    {
	if (depth_data_)
	    delete[] depth_data_;
	row_ = 0;
	col_ = 0;
    }
	
    /// operator overloading
    Depth operator=(const Depth& other) { *this = other; }

    /// get height, width or data ptr of depth image
    inline int rows() const { return row_; }
    inline int cols() const { return col_; }
    inline ushort* data() const { return depth_data_; } 
    
    /// get pesudo-color image of depth image
    uchar* convert_to_color()
    {
	/// color image to be returned
	uchar* rgb_image(nullptr);

	const uchar far_color[] = { 255, 0, 0 }, near_color[] = { 20, 40, 255 };
	const int N = 256 * 256;
	int histogram[N] = { 1 };

	/// histogram
	for (int row_i = 0; row_i < row_; row_i++)
	{
	    for (int col_i = 0; col_i < col_; col_i++)
	    {
		int index = static_cast<int>(depth_data_[row_i * col_ + col_i]);
		histogram[index]++;
	    }
	}

	/// integral arry
	for (int hist_i = 1; hist_i < N; hist_i++)
	    histogram[hist_i] += histogram[hist_i - 1];

	/// remap the integral histogram to range [0...256];
	int sum = histogram[N - 1];
	for (int hist_i = 0; hist_i < N; hist_i++)
	    histogram[hist_i] = (histogram[hist_i] << 8) / sum;

	/// init color image
	rgb_image = new uchar[row_ * col_ * 3];
	memset(rgb_image, 0, sizeof(rgb_image));

	/// generate color image by using the histogram to interpolate between two colors
	for (int row_i = 0; row_i < row_; row_i++)
	{
	    for (int col_i = 0; col_i < col_; col_i++)
	    {
		int pos = row_i * col_ + col_i;

		ushort depth = depth_data_[pos];
		int hist = histogram[depth];

		/// get the corresponding rgb postions in the two dimension color image
		uchar &r(rgb_image[pos * 3]), &g(rgb_image[pos * 3 + 1]), 
			&b(rgb_image[pos * 3 + 2]);
		if (depth > 0)
		{
		    r = ((std::numeric_limits<uchar>::max() - hist) * near_color[0] + hist * far_color[0]) >> 8;
		    g = ((std::numeric_limits<uchar>::max() - hist) * near_color[1] + hist * far_color[1]) >> 8;
		    b = ((std::numeric_limits<uchar>::max() - hist) * near_color[2] + hist * far_color[2]) >> 8;
		}
		else
		{
		    r = static_cast<uchar>(0);
		    g = static_cast<uchar>(0);
		    b = static_cast<uchar>(0);
		}
	    }
	}
	return rgb_image;
    }

private:
    ushort* depth_data_;
    int row_, col_;
};

class FrameDrawer
{
public:
    FrameDrawer( ProjectionParams* projection_params );

    // Update info from the last processed frame.
    void update(Mapping *mapper);
    
    // stitch images
    cv::Mat stitch_images(std::vector<cv::Mat>& srcs);
    
    // Draw last processed frame.
    std::vector<cv::Mat> draw_frame();

protected:

    // Info of the frame to be drawn
    cv::Mat original_depth_image_; // without ground
    cv::Mat angle_diff_image_; // filtered by smooth func
    cv::Mat smoothed_angle_image_;
    cv::Mat segmentation_image_; 
    cv::Mat left_image_;
    cv::Mat right_image_;

    boost::mutex image_mutex_;
};

} //namespace Mobile_Sensing

#endif // FRAME_DRAWER_H
