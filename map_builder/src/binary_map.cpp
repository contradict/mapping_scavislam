#include "map_builder/binary_map.h"

#include <eigen_conversions/eigen_msg.h>
#include <costmap_2d/cost_values.h>

namespace map_builder {

BinaryMap::BinaryMap( costmap_2d::LayeredCostmap *layered, geometry_msgs::PoseStamped pose )
{
    tf::pointMsgToEigen(pose.pose.position, translation_);
    tf::quaternionMsgToEigen(pose.pose.orientation, orientation_);
    tf::quaternionMsgToEigen(pose.pose.orientation, originalOrientation_);
    costmap_2d::Costmap2D *costmap = layered->getCostmap();
    boost::shared_lock<boost::shared_mutex> l( *costmap->getLock() );
    unsigned int x0, xn, y0, yn;
    layered->getBounds(&x0, &xn, &y0, &yn);
    ROS_DEBUG("Create BinaryMap from region (%d, %d)-(%d, %d)", x0, y0, xn,yn);
    sizeX_ = (xn-x0);
    sizeY_ = (yn-y0);
    resolution_ = costmap->getResolution();
    double x, y;
    costmap->mapToWorld( x0, y0, x, y);
    origin_ << x-0.5*resolution_, y-0.5*resolution_, 0.0;
    origin_ -= translation_;

    map_.resize( sizeX_*sizeY_ );
    for( unsigned int y=y0; y<yn; y++)
    {
        for( unsigned int x=x0; x<xn; x++)
        {
            map_[(y-y0)*sizeX_+(x-x0)] = ( costmap->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE );
        }
    }
}

void
BinaryMap::updatePose(geometry_msgs::PoseStamped pose)
{
    tf::quaternionMsgToEigen(pose.pose.orientation, orientation_);
    tf::pointMsgToEigen(pose.pose.position, translation_);
}

void
BinaryMap::getBounds( double& ox, double& oy, double& ex, double& ey)
{
    double x00, y00, x01, y01, x10, y10, x11, y11;
    mapToWorld(      0,      0, x00, y00 );
    mapToWorld( sizeX_,      0, x10, y10 );
    mapToWorld(      0, sizeY_, x01, y01 );
    mapToWorld( sizeX_, sizeY_, x11, y11 );
    ex = std::max(x00, std::max(x01, std::max(x10, x11)));
    ey = std::max(y00, std::max(y01, std::max(y10, y11)));
    ox = std::min(x00, std::min(x01, std::min(x10, x11)));
    oy = std::min(y00, std::min(y01, std::min(y10, y11)));
}

void
BinaryMap::updateBounds( double& ox, double& oy, unsigned int& sizeX, unsigned int& sizeY)
{
    double ex, ey;
    double oox, ooy;
    getBounds( oox, ooy, ex, ey );
    ex = std::max(ex, ox+resolution_*sizeX);
    ey = std::max(ey, oy+resolution_*sizeY);
    ox = std::min(ox, oox);
    oy = std::min(oy, ooy);
    unsigned int sX = ceil((ex-ox)/resolution_);
    unsigned int sY = ceil((ey-oy)/resolution_);
    sizeX = std::max( sX, sizeX );
    sizeY = std::max( sY, sizeY );
    ROS_DEBUG_STREAM("BinaryMap with origin " << oox << " " << ooy <<
                      " size (" << sizeX_ << ", " << sizeY_ << ") " <<
                      " Updates costmap origin to " << ox << " " << oy <<
                      " size (" << sizeX << ", " << sizeY << ") " <<
                      " edge (" << ox+sizeX*resolution_ << ", " <<
                      oy + sizeY*resolution_ << ")"
                      );
};

void
BinaryMap::copyTo( costmap_2d::Costmap2D *c)
{  
    for( unsigned int y=0;y<sizeY_;y++)
    {
        for( unsigned int x=0;x<sizeX_;x++)
        {
            double wx, wy;
            mapToWorld( x, y, wx, wy);
            unsigned int cx, cy;
            if(c->worldToMap( wx, wy, cx, cy))
            {
                if( map_[y*sizeX_+x] )
                {
                    c->setCost( cx, cy, costmap_2d::LETHAL_OBSTACLE );
                }
            }
            else
            {
                ROS_ERROR( "Point outside map (%d, %d)->(%f, %f)", x,y,wx,wy);
                ROS_ERROR( "Map origin (%f, %f) size (%d, %d)",
                        c->getOriginX(),
                        c->getOriginY(),
                        c->getSizeInCellsX(),
                        c->getSizeInCellsY());
            }
        }
    }
}

void
BinaryMap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    Eigen::Vector3d mpos;
    mpos << mx, my, 0.0;
    mpos = orientation_*originalOrientation_.inverse()*
        (origin_+resolution_*(mpos+Eigen::Vector3d(0.5, 0.5, 0.0)))+
        translation_;
    wx = mpos[0];
    wy = mpos[1];
}

void
BinaryMap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
    Eigen::Vector3d mpos;
    mpos << wx, wy, 0.0;
    mpos = (originalOrientation_*orientation_.inverse()*(mpos-translation_) - origin_)/resolution_;
    mx = floor(mpos[0]);
    my = floor(mpos[1]);
}

}
