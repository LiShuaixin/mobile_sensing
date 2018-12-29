#include <frame_drawer.h>

namespace Mobile_Sensing 
{

FrameDrawer::FrameDrawer( ProjectionParams* projection_params )
{
    int row = projection_params->rows();
    int col = projection_params->cols();
    
    original_depth_image_ = cv::Mat(row, col, CV_32FC1, cv::Scalar(0,0,0));
    angle_diff_image_ = cv::Mat(row, col, CV_32FC1, cv::Scalar(0,0,0));
    smoothed_angle_image_ = cv::Mat(row, col, CV_32FC1, cv::Scalar(0,0,0));
    segmentation_image_ = cv::Mat(row, col, CV_32FC1, cv::Scalar(0,0,0));
    left_image_ = cv::Mat(2*row, col, CV_32FC1, cv::Scalar(0,0,0));
    right_image_ = cv::Mat(2*row, col, CV_32FC1, cv::Scalar(0,0,0));
}

void FrameDrawer::update ( Mapping* mapper )
{
    boost::mutex::scoped_lock lock(image_mutex_);
    
    mapper->cur_scan_.scan_projection_->depth_image().copyTo(original_depth_image_); 
    // printf("scan's depth image size: %d * %d.\n", mapper->cur_scan_.scan_projection_->depth_image().rows, mapper->cur_scan_.scan_projection_->depth_image().cols);
    // since I'd like to show the depth image without ground, the frame drawer should be put after ground remover
    
    mapper->depth_ground_remover_->angle_image().copyTo(angle_diff_image_);
    mapper->depth_ground_remover_->smoothed_angle_image().copyTo(smoothed_angle_image_);
    
    // TODO update segmentation image
}

cv::Mat FrameDrawer::stitch_images ( std::vector<cv::Mat>& srcs )
{
    int w = 0, h = 0;
    cv::Mat dst(h, w, CV_32FC1, cv::Scalar(0,0,0));
    for(const auto img : srcs)
    {
	h += img.rows;
	w += img.cols;
	
	cv::Mat tmp(h, w, CV_32FC1, cv::Scalar(0,0,0));
	dst.copyTo(tmp.rowRange(0,dst.rows).colRange(0,dst.cols));
        img.copyTo(tmp.rowRange(dst.rows,h).colRange(0,dst.cols));
	
	tmp.copyTo(dst);
    }
}
 

std::vector<cv::Mat> FrameDrawer::draw_frame()
{
    std::vector<cv::Mat> images;
    images.reserve(4);
    
    cv::Mat original_depth_image, angle_diff_image, smoothed_angle_image, segmentation_image, output;

    //Copy variables within scoped mutex
    {
        boost::mutex::scoped_lock lock(image_mutex_);
        
        original_depth_image_.copyTo(original_depth_image);
	angle_diff_image_.copyTo(angle_diff_image);
	smoothed_angle_image_.copyTo(smoothed_angle_image);
	// TODO sementation image
	segmentation_image_.copyTo(segmentation_image);
    } // destroy scoped mutex -> release mutex
    
    // convert color
    if(original_depth_image.channels()<3) //this should be always true
	cv::cvtColor(original_depth_image,original_depth_image,CV_GRAY2BGR);
    
    if(angle_diff_image.channels()<3) //this should be always true
	cv::cvtColor(angle_diff_image,angle_diff_image,CV_GRAY2BGR);
    
    if(smoothed_angle_image.channels()<3) //this should be always true
	cv::cvtColor(smoothed_angle_image,smoothed_angle_image,CV_GRAY2BGR);
    
    if(segmentation_image.channels()<3) //this should be always true
	cv::cvtColor(segmentation_image,segmentation_image,CV_GRAY2BGR);
    
    // add text
    /*stringstream s1, s2, s3, s4;
    s1 << " ORIGINAL DEPTH IMAGE";
    s2 << " ANGULAR DIFFERENCE IMAGE";
    s3 << " SMOOTHED ANGULAR DIFFERENCE IMAGE";
    s4 << " SEGMENTATION IMAGE";

    int baseline=0;
    cv::Size textsize1 = cv::getTextSize(s1.str(),cv::FONT_HERSHEY_PLAIN,1,0.5,&baseline);
    cv::Size textsize2 = cv::getTextSize(s2.str(),cv::FONT_HERSHEY_PLAIN,1,0.5,&baseline);
    cv::Size textsize3 = cv::getTextSize(s3.str(),cv::FONT_HERSHEY_PLAIN,1,0.5,&baseline);
    cv::Size textsize4 = cv::getTextSize(s4.str(),cv::FONT_HERSHEY_PLAIN,1,0.5,&baseline);

    cv::Mat imtext1, imtext2, imtext3, imtext4;
    imtext1 = cv::Mat(original_depth_image.rows+textsize1.height+5,original_depth_image.cols,original_depth_image.type());
    original_depth_image.copyTo(imtext1.rowRange(0,original_depth_image.rows).colRange(0,original_depth_image.cols));
    imtext1.rowRange(original_depth_image.rows,imtext1.rows) = cv::Mat::zeros(textsize1.height+5,original_depth_image.cols,original_depth_image.type());
    cv::putText(imtext1,s1.str(),cv::Point(2,imtext1.rows-2),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),0.5,8);
    
    imtext2 = cv::Mat(angle_diff_image.rows+textsize2.height+5,angle_diff_image.cols,angle_diff_image.type());
    angle_diff_image.copyTo(imtext2.rowRange(0,angle_diff_image.rows).colRange(0,angle_diff_image.cols));
    imtext2.rowRange(angle_diff_image.rows,imtext2.rows) = cv::Mat::zeros(textsize2.height+5,angle_diff_image.cols,angle_diff_image.type());
    cv::putText(imtext2,s2.str(),cv::Point(2,imtext2.rows-2),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),0.5,8);    
    
    imtext3 = cv::Mat(smoothed_angle_image.rows+textsize3.height+5,smoothed_angle_image.cols,smoothed_angle_image.type());
    smoothed_angle_image.copyTo(imtext3.rowRange(0,smoothed_angle_image.rows).colRange(0,smoothed_angle_image.cols));
    imtext3.rowRange(smoothed_angle_image.rows,imtext3.rows) = cv::Mat::zeros(textsize3.height+5,smoothed_angle_image.cols,smoothed_angle_image.type());
    cv::putText(imtext3,s3.str(),cv::Point(2,imtext3.rows-2),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),0.5,8);   
    
    imtext4 = cv::Mat(segmentation_image.rows+textsize4.height+5,segmentation_image.cols,segmentation_image.type());
    segmentation_image.copyTo(imtext4.rowRange(0,segmentation_image.rows).colRange(0,segmentation_image.cols));
    imtext4.rowRange(segmentation_image.rows,imtext4.rows) = cv::Mat::zeros(textsize4.height+5,segmentation_image.cols,segmentation_image.type());
    cv::putText(imtext4,s3.str(),cv::Point(2,imtext4.rows-2),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),0.5,8);
    
    cv::Mat left_image, right_image;
    stitch_images(imtext1, imtext4, left_image);
    stitch_images(imtext2, imtext3, right_image);
    
    {
        boost::mutex::scoped_lock lock(image_mutex_);
        
        left_image_ = left_image;
        right_image_ = right_image;
    }
    

    Depth depth_image((ushort*)(original_depth_image.data), original_depth_image.rows, original_depth_image.cols);
    uchar* color_image = depth_image.convert_to_color();
    cv::Mat_<cv::Vec3b> display;
    display.create(depth_image.rows(), depth_image.cols());
    std::copy(color_image, color_image + depth_image.rows() * depth_image.cols() * 3, display.data);
    delete[] color_image;
   
    cv::imshow("display", display);
    cv::imshow("1", original_depth_image);
    cv::imshow("2", segmentation_image);
    cv::imshow("3", angle_diff_image);
    cv::imshow("4", smoothed_angle_image);
    cv::waitKey(0);*/

    images.push_back(original_depth_image);
    images.push_back(segmentation_image);
    images.push_back(angle_diff_image);
    images.push_back(smoothed_angle_image);
    
    // printf("DRAW MAP POINTS!\n");
    // output = stitch_images(images);
    // printf("after stitch");
    
    return images;
}

} //namespace Mobile_Sensing
