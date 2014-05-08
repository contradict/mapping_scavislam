
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>

#include "map_builder/binary_map.h"

namespace map_builder {

class Mapper
{
public:
    Mapper(void):
        plugin_loader_("costmap_2d", "costmap_2d::Layer"),
        transientMap_(NULL),
        publishedMap_(NULL),
        publisher_(NULL),
        resolution_(0.1)
    {};
    ~Mapper(void);
    void init(void);
    void pathCallback( nav_msgs::Path::ConstPtr msg );

private:

    void publishMap(void);

    pluginlib::ClassLoader<costmap_2d::Layer> plugin_loader_;

    ros::Subscriber slam_pose_sub_;

    std::map< ros::Time, BinaryMap > maps_;

    tf::TransformListener listener_;

    costmap_2d::Costmap2DROS* transientMap_;
    costmap_2d::Costmap2D* publishedMap_;
    costmap_2d::Costmap2DPublisher *publisher_;

    double resolution_;
};

}
