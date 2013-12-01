
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include "slamgraph_visual.h"

namespace scavislam_rviz
{

// BEGIN_TUTORIAL
SLAMGraphVisual::SLAMGraphVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  points_.reset(new rviz::PointCloud());
  frame_node_->attachObject(points_.get());
}

SLAMGraphVisual::~SLAMGraphVisual()
{
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode( frame_node_ );
}

void SLAMGraphVisual::setMessage( const scavislam_messages::SLAMGraph::ConstPtr& msg )
{
  rviz::PointCloud::Point *newpoints = new rviz::PointCloud::Point[msg->points.size()];
  int i=0;
  for(const auto point :msg->points) {
      newpoints[i].color = point_color_;
      newpoints[i].position[0]=point.x;
      newpoints[i].position[1]=point.y;
      newpoints[i++].position[2]=point.z;
  }
  points_->clear();
  points_->addPoints(newpoints, i);
  delete[] newpoints;

  for(const auto vertex : msg->vertices){
      boost::shared_ptr<rviz::Axes> vert(new rviz::Axes( scene_manager_, frame_node_ ));
      Ogre::Vector3 pos(vertex.pose.position.x,
              vertex.pose.position.y,
              vertex.pose.position.z);
      vert->setPosition(pos);
      Ogre::Quaternion orient(vertex.pose.orientation.w,
                              vertex.pose.orientation.x,
                              vertex.pose.orientation.y,
                              vertex.pose.orientation.z);
      vert->setOrientation(orient);
      vertices_.push_back(vert);
  }

  for(const auto vertex : msg->vertices){
      for(const auto point : vertex.points){
          boost::shared_ptr<rviz::BillboardLine> eln(new rviz::BillboardLine( scene_manager_, frame_node_ ));
          Ogre::Vector3 p0(vertex.pose.position.x,
                           vertex.pose.position.y,
                           vertex.pose.position.z);
          Ogre::Vector3 p1(msg->vertices.at(point.data).pose.position.x,
                           msg->vertices.at(point.data).pose.position.y,
                           msg->vertices.at(point.data).pose.position.z);
          eln->addPoint(p0);
          eln->addPoint(p1);
          edges_.push_back(eln);
      }
  }

}

// Position and orientation are passed through to the SceneNode.
void SLAMGraphVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void SLAMGraphVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

// Color is passed through to the Arrow object.
void SLAMGraphVisual::setColor(Ogre::ColourValue &vertexcolor,
                               Ogre::ColourValue &edgecolor,
                               Ogre::ColourValue &pointcolor,
                               float a )
{
  point_color_ = pointcolor;
}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

