#include <map>
#include <memory>
#include <limits>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "map_builder/mapper.h"

namespace map_builder {

Mapper::~Mapper(void)
{
    if( publisher_ != NULL )
        delete publisher_;
    if( transientMap_ != NULL )
        delete transientMap_;
    if( publishedMap_ != NULL )
        delete publishedMap_;
}

void Mapper::init( void )
{
    ros::NodeHandle nh_;

    ros::NodeHandle private_nh("~");

    publishedMap_ = new costmap_2d::Costmap2D();

    publisher_ = new costmap_2d::Costmap2DPublisher( &private_nh, publishedMap_, "map", "published_map");

    transientMap_ = new costmap_2d::Costmap2DROS("transient_map", listener_);


    transientMap_->start();

    slam_pose_sub_ = nh_.subscribe("slam_path", 1, &Mapper::pathCallback, this);
}

void Mapper::pathCallback( nav_msgs::Path::ConstPtr msg )
{
    geometry_msgs::PoseStamped last;
    last.header.stamp = ros::Time(0);
    for( auto pose : msg->poses )
    {
        auto mapit = maps_.find( pose.header.stamp );
        if(mapit == maps_.end())
        {
            if(pose.header.stamp > last.header.stamp)
            {
                last = pose;
            }
        }
        else
        {
            mapit->second.updatePose( pose );
            ROS_DEBUG_STREAM("Updating pose from " << mapit->first );
        }
    }

    if( last.header.stamp != ros::Time(0) )
    {
        ROS_INFO_STREAM( "Appending new pose " << last.header.stamp );
        transientMap_->updateMap();
        costmap_2d::LayeredCostmap *layered = transientMap_->getLayeredCostmap();
        BinaryMap m(layered, last);
        transientMap_->resetLayers();
        maps_.insert( std::pair<ros::Time, BinaryMap>(last.header.stamp, m) );
        publishMap();
    }
}

void Mapper::publishMap( void )
{
    
    double ox, oy;
    maps_.begin()->second.mapToWorld( 0, 0, ox, oy);
    unsigned int sizeX=maps_.begin()->second.sizeX_;
    unsigned int sizeY=maps_.begin()->second.sizeY_;

    std::tie(ox, oy, sizeX, sizeY) =
        std::accumulate( maps_.begin(), maps_.end(), std::make_tuple( ox, oy, sizeX, sizeY ),
            [](std::tuple<double,double,unsigned int,unsigned int> &bounds,
                std::pair< ros::Time, BinaryMap> t_map)
            {
                double ox, oy;
                unsigned int sX, sY;
                std::tie( ox, oy, sX, sY ) = bounds;
                t_map.second.updateBounds( ox, oy, sX, sY );
                return std::make_tuple(ox, oy, sX, sY);
            });

    publishedMap_->resizeMap(sizeX+3, sizeY+4, resolution_, ox, oy);

    std::accumulate( maps_.begin(), maps_.end(), publishedMap_,
            [](costmap_2d::Costmap2D *c, std::pair<const ros::Time, BinaryMap > &p)
            {
                p.second.copyTo( c );
                return c;
            });

    ROS_INFO( "Publish global map" );
    publisher_->publishCostmap();
}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MapCreator");

    map_builder::Mapper node;
    node.init();

    ROS_INFO("Spin");
    ros::spin();

    return 0;
}

