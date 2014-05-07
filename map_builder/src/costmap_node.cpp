#include <map>
#include <memory>
#include <limits>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mapping_scavislam
{
struct BinaryMap
{
    BinaryMap( costmap_2d::LayeredCostmap *layered, geometry_msgs::PoseStamped pose )
    {
        tf::pointMsgToEigen(pose.pose.position, originalTranslation_);
        tf::quaternionMsgToEigen(pose.pose.orientation, originalOrientation_);
        orientation_.setIdentity();
        translation_.setZero();
        costmap_2d::Costmap2D *costmap = layered->getCostmap();
        boost::shared_lock<boost::shared_mutex> l( *costmap->getLock() );
        unsigned int x0, xn, y0, yn;
        layered->getBounds(&x0, &xn, &y0, &yn);
        sizeX_ = (xn-x0);
        sizeY_ = (yn-y0);
        resolution_ = costmap->getResolution();
        origin_[0] = costmap->getOriginX();
        origin_[1] = costmap->getOriginY();
        double x, y;
        costmap->mapToWorld( x0, y0, x, y);
        upperLeft_ << x,y;
        costmap->mapToWorld( xn, yn, x, y);
        lowerRight_ << x,y;

        map_.resize( sizeX_*sizeY_ );
        for( unsigned int y=y0; y<yn; y++)
        {
            for( unsigned int x=x0; x<xn; x++)
            {
                map_[(y-y0)*sizeX_+(x-x0)] = ( costmap->getCost(x, y) == 254 );
            }
        }
    };

    std::pair< double, double > updatePose(geometry_msgs::PoseStamped pose)
    {
        Eigen::Quaterniond q;
        Eigen::Vector3d v;
        tf::quaternionMsgToEigen(pose.pose.orientation, q);
        tf::pointMsgToEigen(pose.pose.position, v);
        Eigen::Quaterniond orig_orientation = orientation_;
        Eigen::Vector3d orig_translation = translation_;
        orientation_ = q*orientation_.inverse();
        translation_ = v-translation_;
        Eigen::Quaterniond dq = orientation_*orig_orientation.inverse();
        return std::make_pair( (translation_ - orig_translation).norm(),
                               2*acos(dq.w()));
    };

    void updateBounds( Eigen::Array2d& upperLeft, Eigen::Array2d& lowerRight)
    {
        upperLeft = upperLeft.min(upperLeft_);
        lowerRight = upperLeft.max(lowerRight_);
    };

    void copyTo( costmap_2d::Costmap2D *c)
    {
        for( unsigned int y=0;y<sizeY_;y++)
        {
            for( unsigned int x=0;x<sizeX_;x++)
            {
                double wx, wy;
                mapToWorld( x, y, wx, wy);
                unsigned int cx, cy;
                c->worldToMap( wx, wy, cx, cy);
                if( map_[y*sizeX_+x] )
                {
                    c->setCost( cx, cy, 254 );
                }
            }
        }
    };

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
    {
        Eigen::Vector3d mpos;
        mpos << (mx+0.5)*resolution_,
                (my+0.5)*resolution_,
                0.0;
        mpos = orientation_*mpos;
        wx = origin_[0] + translation_[0] + mpos[0];
        wy = origin_[1] + translation_[1] + mpos[1];
    };

    Eigen::Vector2d origin_;
    Eigen::Vector3d translation_;
    Eigen::Vector3d originalTranslation_;
    Eigen::Quaterniond orientation_;
    Eigen::Quaterniond originalOrientation_;
    Eigen::Array2d upperLeft_, lowerRight_;
    double resolution_;
    double sizeX_, sizeY_;
    std::vector<bool> map_;
};

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

    Eigen::Array2d upperLeft_, lowerRight_;

    double resolution_;
};

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

    slam_pose_sub_ = nh_.subscribe("slam_path", 1, &Mapper::pathCallback, this);

    ros::NodeHandle private_nh("~");

    publishedMap_ = new costmap_2d::Costmap2D();

    publisher_ = new costmap_2d::Costmap2DPublisher( &private_nh, publishedMap_, "map", "published_map");

    transientMap_ = new costmap_2d::Costmap2DROS("transient_map", listener_);

    upperLeft_ << std::numeric_limits<double>::max(),std::numeric_limits<double>::max();
    lowerRight_ << std::numeric_limits<double>::min(),std::numeric_limits<double>::min();

    transientMap_->start();
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
            double dx, dq;
            std::tie( dx, dq ) = mapit->second.updatePose( pose );
            ROS_DEBUG_STREAM("Updating pose from " << mapit->first
                    << " (" << dx << ", " << dq << ")");
        }
    }

    if( last.header.stamp != ros::Time(0) )
    {
        ROS_INFO_STREAM( "Appending new pose " << last.header.stamp );
        transientMap_->updateMap();
        costmap_2d::LayeredCostmap *layered = transientMap_->getLayeredCostmap();
        BinaryMap m(layered, last);
        m.updateBounds( upperLeft_, lowerRight_ );
        transientMap_->resetLayers();
        maps_.insert( std::pair<ros::Time, BinaryMap>(last.header.stamp, m) );
        publishMap();
    }
}

void Mapper::publishMap( void )
{
    unsigned int sizeX = round(fabs((lowerRight_[0] - upperLeft_[0])/resolution_));
    unsigned int sizeY = round(fabs((lowerRight_[1] - upperLeft_[1])/resolution_));
    Eigen::Array2d origin = (upperLeft_ + lowerRight_)/2.;
    publishedMap_->resizeMap(sizeX, sizeY, resolution_, origin[0], origin[1]);

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

    mapping_scavislam::Mapper node;
    node.init();

    ROS_INFO("Spin");
    ros::spin();

    return 0;
}

