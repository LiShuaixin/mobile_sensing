#ifndef VIEWER_H
#define VIEWER_H

#include <frame_drawer.h>
#include <map_drawer.h>
#include <mapping.h>
#include <system.h>

#include <pangolin/pangolin.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

namespace Mobile_Sensing
{

class Mapping;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{
public:
    Viewer( System* system, FrameDrawer* frame_drawer, MapDrawer* map_drawer, Mapping *mapper );
    
    Viewer( const Viewer& viewer );

    // Main thread function. Draw pointcloud, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void visualize();

    void request_finish();

    void request_stop();

    bool is_finished();

    bool is_stopped();

    void release();

public:

    bool stop();

    System* system_;
    FrameDrawer* frame_drawer_;
    MapDrawer* map_drawer_;
    Mapping* mapper_;

    // 1/fps in ms
    double T_;
    float image_width_, image_height_;

    float viewpointX_, viewpointY_, viewpointZ_, viewpointF_;

    bool check_finish();
    void set_finish();
    bool finish_requested_;
    bool finished_;
    boost::mutex finish_mutex_;

    bool stopped_;
    bool stop_requested_;
    boost::mutex stop_mutex_;

};

}

#endif // VIEWER_H
	

