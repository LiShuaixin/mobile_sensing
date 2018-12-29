#include <viewer.h>

namespace Mobile_Sensing
{

Viewer::Viewer ( System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer, Mapping* mapper ) :
    system_(system), frame_drawer_(frame_drawer), map_drawer_(map_drawer), mapper_(mapper),
    finish_requested_(false), finished_(true), stopped_(true), stop_requested_(false)
{
    float fps = 10.;
    T_ = 1e3/fps;

    image_width_ = mapper_->projection_params_->cols();
    image_height_ = mapper_->projection_params_->rows();

    viewpointX_ = 0;
    viewpointY_ = -10;
    viewpointZ_ = -0.1;
    viewpointF_ = 2000;
}

Viewer::Viewer ( const Viewer& viewer ) : system_(viewer.system_), frame_drawer_(viewer.frame_drawer_), 
        map_drawer_(viewer.map_drawer_), mapper_(viewer.mapper_), T_(viewer.T_), image_width_(viewer.image_width_), 
        image_height_(viewer.image_height_), viewpointX_(viewer.viewpointX_), viewpointY_(viewer.viewpointY_), 
        viewpointZ_(viewer.viewpointZ_), viewpointF_(viewer.viewpointF_) {}

void Viewer::visualize()
{
    printf("START PANGOLIN!\n");
    
    finished_ = false;
    stopped_ = false;

    float r = mapper_->projection_params_->rows();
    float c = mapper_->projection_params_->cols();
    
    pangolin::CreateWindowAndBind("Mobile Sensing", 1870, 522);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175)); // 设置面板纵向高度与窗口大小相同，右边横向175个像素的范围显示面板
    pangolin::Var<bool> menu_follow_camera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menu_show_points("menu.Show Points",true,true);
    pangolin::Var<bool> menu_show_trajectory("menu.Show Trajectory",true,true);
    pangolin::Var<bool> menu_reset("menu.Reset",false,false);
    
    // Define LiDAR Render Object (for view / scene browsing)
    /**
     * 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
     * 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
     * 观测目标位置：(0, 0, 0)
     * 观测的方位向量：(0.0,-1.0, 0.0)*/
    pangolin::OpenGlRenderState visualization3D_lidar(
                pangolin::ProjectionMatrix(870,522,viewpointF_,viewpointF_,435,261,0.1,1000),
                pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0)
                ); 

    // Add named OpenGL viewport to window and provide 3D Handler
    /**
     * 定义地图面板
     * SetBound最后一个参数（-1740.0f/1044.0f）为显示长宽比
     */
    pangolin::View& Visualization3D_display = pangolin::CreateDisplay()
	    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -870.0f/522.0f)
	    .SetHandler(new pangolin::Handler3D(visualization3D_lidar)); 

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    // 3 images
    pangolin::View& img_depth = pangolin::Display("Depth")
	.SetAspect(-c/(float)r);

    pangolin::View& img_segmentation = pangolin::Display("Segmentation")
	.SetAspect(-c/(float)r);
	
    pangolin::View& img_angular = pangolin::Display("Angular")
	.SetAspect(-c/(float)r);

    pangolin::View& img_smoothed = pangolin::Display("Smoothed")
	.SetAspect(-c/(float)r);
	
    pangolin::GlTexture tex_depth(c,r,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture tex_segmentation(c,r,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);  
    pangolin::GlTexture tex_angular(c,r,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture tex_smoothed(c,r,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
	
    pangolin::CreateDisplay()
	.SetBounds(0.0, 0.23, pangolin::Attach::Pix(175), 1.0)
	.SetLayout(pangolin::LayoutEqual)
	.AddDisplay(img_depth)
	.AddDisplay(img_segmentation)
	.AddDisplay(img_angular)
	.AddDisplay(img_smoothed);

    bool follow = true;

    while(1)
    {
	// Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
	// draw 3D lcoal map
        map_drawer_->get_current_OpenGL_lidar_matrix(Twc);
	// printf("SET LIDAR POSE!\n");

        if(menu_follow_camera && follow)
        {
            visualization3D_lidar.Follow(Twc);
        }
        else if(menu_follow_camera && !follow)
        {
            visualization3D_lidar.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0));
            visualization3D_lidar.Follow(Twc);
            follow = true;
        }
        else if(!menu_follow_camera && follow)
        {
            follow = false;
        }

        Visualization3D_display.Activate(visualization3D_lidar);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        map_drawer_->draw_current_lidar(Twc);
        if(menu_show_trajectory)
            map_drawer_->draw_trajectory();
	// printf("VIEW:DRAW TRAJECTORY!\n");
        if(menu_show_points)
            map_drawer_->draw_map_points();
	// printf("VIEW:DRAW MAP POINTS!\n");
	
	// draw images
	std::vector<cv::Mat> images = frame_drawer_->draw_frame();
	if(images.size() != 4)
	    printf("SOMETHING COULD BE WRONG IN FRAMEDRAWER!\n");

	/*for(int i = 0; i < images.size(); i++)
	{
	    if(images[i].channels()>1) //this should be always true
		cv::cvtColor(images[i],images[i],CV_BGR2GRAY);   
	}*/
	
	tex_depth.Upload(images[0].data,GL_BGR,GL_FLOAT);
	tex_segmentation.Upload(images[1].data,GL_BGR,GL_FLOAT);
	tex_angular.Upload(images[2].data,GL_BGR,GL_FLOAT);
	tex_smoothed.Upload(images[3].data,GL_BGR,GL_FLOAT);
	
	img_depth.Activate();
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	tex_depth.RenderToViewportFlipY();
	
	img_segmentation.Activate();
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	tex_segmentation.RenderToViewportFlipY();
	
	img_angular.Activate();
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	tex_angular.RenderToViewportFlipY();
	
	img_smoothed.Activate();
	glColor4f(1.0f,1.0f,1.0f,1.0f);
	tex_smoothed.RenderToViewportFlipY();
	        
	pangolin::FinishFrame();
        
        if(menu_reset)
        {
            menu_show_trajectory = true;
            menu_show_points = true;
            follow = true;
            menu_follow_camera = true;
            menu_reset = false;
        }

        if(stop())
        {
            while(is_stopped())
            {
                usleep(3000);
            }
        }

        if(check_finish())
            break;
    }
    printf("END LOOP\n");
    
    set_finish();
}

void Viewer::request_finish()
{
    boost::mutex::scoped_lock lock(finish_mutex_);
    finish_requested_ = true;
}

bool Viewer::check_finish()
{
    boost::mutex::scoped_lock lock(finish_mutex_);
    return finish_requested_;
}

void Viewer::set_finish()
{
    boost::mutex::scoped_lock lock(finish_mutex_);
    finished_ = true;
}

bool Viewer::is_finished()
{
    boost::mutex::scoped_lock lock(finish_mutex_);
    return finished_;
}

void Viewer::request_stop()
{
    boost::mutex::scoped_lock lock(stop_mutex_);
    
    // the requestion can only be changed when the viewer is running
    if(!stopped_)
        stop_requested_ = true;
}

bool Viewer::is_stopped()
{
    boost::mutex::scoped_lock lock(stop_mutex_);
    return stopped_;
}

bool Viewer::stop()
{
    boost::mutex::scoped_lock lock_stop(stop_mutex_);
    boost::mutex::scoped_lock lock_finish(finish_mutex_);

    if(finish_requested_)
        return false;
    else if(stop_requested_)
    {
        stopped_ = true;
        stop_requested_ = false;
        return true;
    }

    return false;
}

void Viewer::release()
{
    boost::mutex::scoped_lock lock_stop(stop_mutex_);
    stopped_ = false;
}

}
