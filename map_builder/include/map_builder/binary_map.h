
#include <Eigen/Dense>

#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseStamped.h>

namespace map_builder
{

struct BinaryMap
{
    BinaryMap( costmap_2d::LayeredCostmap *layered, geometry_msgs::PoseStamped pose );
    void updatePose(geometry_msgs::PoseStamped pose);
    void getBounds( double& ox, double& oy, double& ex, double& ey);
    void updateBounds( double& ox, double& oy, unsigned int& sizeX, unsigned int& sizeY);
    void copyTo( costmap_2d::Costmap2D *c);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    void worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

    Eigen::Vector3d translation_;
    Eigen::Quaterniond orientation_;
    Eigen::Quaterniond originalOrientation_;
    Eigen::Vector3d origin_;
    double resolution_;
    unsigned int sizeX_, sizeY_;
    std::vector<bool> map_;
};

}
