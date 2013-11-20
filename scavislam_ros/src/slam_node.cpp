#ifdef SCAVISLAM_CUDA_SUPPORT
#include <cutil_inline.h>
#endif

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/stereo_camera_model.h>

#include <visiontools/accessor_macros.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <nav_msgs/Path.h>

#include <scavislam_ros/StereoVSLAMConfig.h>
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


#include <opencv2/calib3d/calib3d.hpp>
//#include <visiontools/accessor_macros.h>

//#include "global.h"

#include "backend.h"
#include "stereo_camera.h"
//#include "framedata.hpp"
#include "placerecognizer.h"
#include "stereo_frontend.h"

namespace stereo_vslam_node {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace ScaViSLAM;

class StereoVSLAMNode
{
    public:
        StereoVSLAMNode();
        ~StereoVSLAMNode();
        void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
                const ImageConstPtr& r_image_msg,const CameraInfoConstPtr& r_info_msg);
        void InitROS();

    private:

        boost::shared_ptr<image_transport::ImageTransport> it_;

        // Subscriptions
        image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
        message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
        typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
        typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        boost::shared_ptr<ExactSync> exact_sync_;
        boost::shared_ptr<ApproximateSync> approximate_sync_;

        // Publications
        ros::Publisher pub_disparity_;
        ros::Publisher pub_neighborhood_points_;
        ros::Publisher pub_pose_history_;

        tf::TransformBroadcaster pose_;

        // Dynamic reconfigure
        boost::recursive_mutex config_mutex_;
        typedef stereo_vslam_node::StereoVSLAMConfig Config;
        typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
        Config config;
        boost::shared_ptr<ReconfigureServer> reconfigure_server_;

        // Processing state (note: only safe because we're single-threaded!)
        image_geometry::StereoCameraModel model_;
        pangolin::DataLog logger;
        StereoCamera *stereo_camera;
        FrameData<StereoCamera> *frame_data;
        StereoFrontend * frontend;
        Backend * backend;
        PerformanceMonitor * per_mon;
        PlaceRecognizer * pla_reg;
        bool vslam_init_;
        cv::Ptr<cv::gpu::FilterEngine_GPU> dx_filter_;
        cv::Ptr<cv::gpu::FilterEngine_GPU> dy_filter_;

        void InitVSLAM();
        void processNextFrame(cv::Mat image_l, cv::Mat image_r);

        void fillDisparityGpu(DisparityImagePtr disp_msg);
        void publishDisparity(
                const ImageConstPtr& l_image_msg,
                const CameraInfoConstPtr& l_info_msg
                );
        void publishNeighborhood(
                const NeighborhoodPtr neighborhood,
                const CameraInfoConstPtr& l_info_msg
                );

        void configCb(Config &config, uint32_t level);
};

StereoVSLAMNode::StereoVSLAMNode()
    :
    stereo_camera(NULL),
    frame_data(NULL),
    frontend(NULL),
    backend(NULL),
    per_mon(NULL),
    pla_reg(NULL),
    vslam_init_(false)
{}

StereoVSLAMNode::~StereoVSLAMNode()
{
    if(stereo_camera!=NULL)
        delete stereo_camera;
    if(frame_data!=NULL)
        delete frame_data;
    if(frontend!=NULL)
        delete frontend;
    if(backend!=NULL)
        delete backend;
    if(per_mon!=NULL)
        delete per_mon;
    if(pla_reg!=NULL)
        delete pla_reg;
}


void StereoVSLAMNode::InitROS()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    it_.reset(new image_transport::ImageTransport(nh));

    // Set up dynamic reconfiguration
    ReconfigureServer::CallbackType f = boost::bind(&StereoVSLAMNode::configCb,
            this, _1, _2);
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
    reconfigure_server_->setCallback(f);

    private_nh.param("wordspath", config.wordspath, string("data/surfwords10000.png"));
    // Synchronize inputs. Topic subscriptions happen on demand in the connection
    // callback. Optionally do approximate synchronization.
    private_nh.param("queue_size", config.queue_size, 5);
    bool approx;
    private_nh.param("approximate_sync", config.approximate_sync, false);
    if (config.approximate_sync)
    {
        approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(config.queue_size),
                    sub_l_image_, sub_l_info_,
                    sub_r_image_, sub_r_info_) );
        approximate_sync_->registerCallback(boost::bind(&StereoVSLAMNode::imageCb,
                    this, _1, _2, _3, _4));
    }
    else
    {
        exact_sync_.reset( new ExactSync(ExactPolicy(config.queue_size),
                    sub_l_image_, sub_l_info_,
                    sub_r_image_, sub_r_info_) );
        exact_sync_->registerCallback(boost::bind(&StereoVSLAMNode::imageCb,
                    this, _1, _2, _3, _4));
    }

    pub_disparity_ = nh.advertise<DisparityImage>("gpu_disparity", 1 );
    pub_neighborhood_points_ = nh.advertise<sensor_msgs::PointCloud2>("neighborhood_points", 1);
    pub_pose_history_ = nh.advertise<nav_msgs::Path>("path", 1);

    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(), private_nh);
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
}

void StereoVSLAMNode::InitVSLAM()
{
    cv::Size res=model_.right().reducedResolution();
    int cam_width=res.width;
    int cam_height=res.height;
    double cam_f=model_.right().fx();
    double cam_px=model_.right().cx();
    double cam_py=model_.right().cy();
    double cam_baseline=model_.baseline();

    StereoCamera stereo_camera((double)cam_f,
            Vector2d((double)cam_px,(double)cam_py),
            cv::Size((int)cam_width,(int)cam_height),
            cam_baseline);

    per_mon = new PerformanceMonitor;
    per_mon->add("drawing");
    per_mon->add("back end");
    per_mon->add("grab frame");
    per_mon->add("preprocess");
    per_mon->add("stereo");
    per_mon->add("dense tracking");
    per_mon->add("fast");
    per_mon->add("match");
    per_mon->add("process points");
    per_mon->add("drop keyframe");
    per_mon->add("dense point cloud");
    per_mon->setup(&logger);

    frame_data = new FrameData<StereoCamera>(stereo_camera);

  dx_filter_ =
      cv::gpu::createDerivFilter_GPU(frame_data->cur_left()
                                     .gpu_pyr_float32[0].type(),
                                     frame_data->gpu_pyr_float32_dx[0]
                                     .type(),
                                     1, 0, 1, cv::BORDER_REPLICATE);
  dy_filter_ =
      cv::gpu::createDerivFilter_GPU(frame_data->cur_left()
                                     .gpu_pyr_float32[0].type(),
                                     frame_data->gpu_pyr_float32_dy[0]
                                     .type(),
                                     0, 1, 1, cv::BORDER_REPLICATE);


    frontend =
        new StereoFrontend(frame_data, per_mon);
    frontend->initialize();

    pla_reg = new PlaceRecognizer(stereo_camera, config.wordspath);
    backend
        = new Backend(frame_data->cam_vec,
                &pla_reg->monitor);
    boost::thread place_reg_thread(boost::ref(*pla_reg));
    boost::thread backend_thread(boost::ref(*backend));

    vslam_init_ = true;
}

void StereoVSLAMNode::
processNextFrame(cv::Mat image_l, cv::Mat image_r)
{
  frame_data->nextFrame();

  frame_data->cur_left().uint8 = image_l;

  frame_data->right.uint8 = image_r;

  per_mon->start("preprocess");
  cv::buildPyramid(frame_data->cur_left().uint8,
                   frame_data->cur_left().pyr_uint8,
                   NUM_PYR_LEVELS-1);

  frame_data->cur_left().gpu_uint8.upload(frame_data->cur_left().uint8);
  frame_data->right.gpu_uint8.upload(frame_data->right.uint8);
  frame_data->cur_left().gpu_uint8
      .convertTo(frame_data->cur_left().gpu_pyr_float32[0], CV_32F,1./255.);

  dx_filter_->apply(frame_data->cur_left().gpu_pyr_float32[0],
                    frame_data->gpu_pyr_float32_dx[0]);
  dy_filter_->apply(frame_data->cur_left().gpu_pyr_float32[0],
                    frame_data->gpu_pyr_float32_dy[0]);

  for (int l=1; l<NUM_PYR_LEVELS; ++l)
  {
    cv::gpu::pyrDown(frame_data->cur_left().gpu_pyr_float32[l-1],
                     frame_data->cur_left().gpu_pyr_float32[l]);
    dx_filter_->apply(frame_data->cur_left().gpu_pyr_float32[l],
                      frame_data->gpu_pyr_float32_dx[l]);
    dy_filter_->apply(frame_data->cur_left().gpu_pyr_float32[l],
                      frame_data->gpu_pyr_float32_dy[l]);
  }
  per_mon->stop("preprocess");

  ++frame_data->frame_id;

}

void StereoVSLAMNode::configCb(Config &newconfig, uint32_t level)
{
    config = newconfig;
    ROS_INFO("New configuration:\n  wordspath: %s\n  queue_size: %d\n  approximate_sync: %d",
            config.wordspath.c_str(),
            config.queue_size,
            config.approximate_sync);
}

void StereoVSLAMNode::imageCb(
        const ImageConstPtr& l_image_msg,
        const CameraInfoConstPtr& l_info_msg,
        const ImageConstPtr& r_image_msg,
        const CameraInfoConstPtr& r_info_msg)
{
    per_mon->new_frame();
    /// @todo Convert (share) with new cv_bridge
    assert(l_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
    assert(r_image_msg->encoding == sensor_msgs::image_encodings::MONO8);

    // Update the camera model
    model_.fromCameraInfo(l_info_msg, r_info_msg);


    // Create cv::Mat views onto all buffers
    const cv::Mat_<uint8_t> l_image(l_image_msg->height, l_image_msg->width,
            const_cast<uint8_t*>(&l_image_msg->data[0]),
            l_image_msg->step);
    const cv::Mat_<uint8_t> r_image(r_image_msg->height, r_image_msg->width,
            const_cast<uint8_t*>(&r_image_msg->data[0]),
            r_image_msg->step);

    if (not vslam_init_)
    {
        InitVSLAM();
        processNextFrame(l_image, r_image);
        frontend->processFirstFrame();
        assert(frontend->to_optimizer_stack.size()==1);
        backend->monitor
            .pushKeyframe(frontend->to_optimizer_stack.top());
        frontend->to_optimizer_stack.pop();
        return;
    }

    float ui_fps = per_mon->fps();
    processNextFrame(l_image, r_image);
    NeighborhoodPtr neighborhood;
    backend->monitor.queryNeighborhood(frontend->actkey_id);

    if (backend->monitor.getNeighborhood(&neighborhood))
    {
        if (neighborhood->vertex_map.find(frontend->actkey_id)
                != neighborhood->vertex_map.end())
        {
            frontend->neighborhood() = neighborhood;
            publishNeighborhood(neighborhood, l_info_msg);
        }
    }
    bool is_frame_droped = false;
    bool tracking_worked = frontend->processFrame(&is_frame_droped);
    publishDisparity(l_image_msg, l_info_msg);
    if(tracking_worked==false)
    {
        return;
    }
    if (is_frame_droped)
    {
        assert(frontend->to_optimizer_stack.size()==1);
        AddToOptimzerPtr to_opt = frontend->to_optimizer_stack.top();
        backend->monitor.pushKeyframe(to_opt);
        frontend->to_optimizer_stack.pop();
    }
    DetectedLoop loop;
    bool is_loop_detected = backend->monitor.getClosedLoop(&loop);
    /*
    if (is_loop_detected)
    {
        best_match = loop.loop_keyframe_id;
    }
    */
}


void StereoVSLAMNode::publishDisparity(
        const ImageConstPtr& l_image_msg,
        const CameraInfoConstPtr& l_info_msg
        )
{
    if( pub_disparity_.getNumSubscribers() == 0 )
        return;

    DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
    disp_msg->header         = l_info_msg->header;
    disp_msg->image.header   = l_info_msg->header;
    disp_msg->image.height   = l_image_msg->height;
    disp_msg->image.width    = l_image_msg->width;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
    disp_msg->f = model_.right().fx();
    disp_msg->T = model_.baseline();

    int windowSize, minDisparity, num_disparities;
    frontend->getDisparityParameters(&windowSize, &minDisparity, &num_disparities);
    int border = windowSize / 2;
    int left = num_disparities + minDisparity + border - 1;
    int wtf = (minDisparity >= 0) ? border + minDisparity : std::max(border, -minDisparity);
    int right = disp_msg->image.width - 1 - wtf;
    int top = border;
    int bottom = disp_msg->image.height - 1 - border;

    disp_msg->valid_window.x_offset = left;
    disp_msg->valid_window.y_offset = top;
    disp_msg->valid_window.width    = right - left;
    disp_msg->valid_window.height   = bottom - top;

    // Disparity search range
    disp_msg->min_disparity = minDisparity;
    disp_msg->max_disparity = minDisparity + num_disparities - 1;
    disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

    cv_bridge::CvImage cv;
    cv.image = frame_data->disp;
    cv.toImageMsg(disp_msg->image);
    disp_msg->image.header   = l_info_msg->header;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
    /*
       double cx_l = model_.left().cx();
       double cx_r = model_.right().cx();
       if (cx_l != cx_r)
       cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
       */
    pub_disparity_.publish(disp_msg);
}

void StereoVSLAMNode::publishNeighborhood(
        const NeighborhoodPtr neighborhood,
        const CameraInfoConstPtr& l_info_msg
        )
{
    pcl::PointCloud<pcl::PointXYZ> neighborhood_cloud;

    for (list<CandidatePoint3Ptr>::const_iterator
         it=neighborhood->point_list.begin();
         it!=neighborhood->point_list.end(); ++it)
    {
      const CandidatePoint3Ptr & ap = *it;
      Eigen::Vector3d pt(GET_MAP_ELEM(ap->anchor_id, neighborhood->vertex_map).T_me_from_w.inverse()
                      *ap->xyz_anchor);
      neighborhood_cloud.push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
    }
    /*
    Draw3d::points(points,2);
    for (ALIGNED<FrontendVertex>::int_hash_map::const_iterator
         it=neighborhood->vertex_map.begin();
         it!=neighborhood->vertex_map.end(); ++it)
    {
      SE3d T1 = it->second.T_me_from_w.inverse();
      Draw3d::pose(T1);

      for (multimap<int,int>::const_iterator it2 =
           it->second.strength_to_neighbors.begin();
           it2 != it->second.strength_to_neighbors.end(); ++it2)
      {
        SE3d T2 = GET_MAP_ELEM(it2->second,
                              neighborhood->vertex_map)
            .T_me_from_w.inverse();
        Draw3d::line(T1.translation(), T2.translation());
      }
    }
    */
    pcl::PCLPointCloud2 pub_cloud;
    pcl::toPCLPointCloud2(neighborhood_cloud, pub_cloud);
    pub_cloud.header.seq = l_info_msg->header.seq;
    pub_cloud.header.stamp = l_info_msg->header.stamp.toNSec();
    pub_cloud.header.frame_id = l_info_msg->header.frame_id;
    pub_neighborhood_points_.publish(pub_cloud);
}

} // namespace stereo_vslam_node




//TODO: main method to long...
int main(int argc, char** argv)
{

#ifdef SCAVISLAM_CUDA_SUPPORT
    cudaDeviceProp prop;
    CUDA_SAFE_CALL( cudaGetDeviceProperties(&prop, 0) );
    std::cout << "Multiprocessors: " << prop.multiProcessorCount << std::endl;
#endif

    ros::init(argc, argv, "StereoVSLAM");

    stereo_vslam_node::StereoVSLAMNode node;
    node.InitROS();

    ros::spin();

    return 0;
}

