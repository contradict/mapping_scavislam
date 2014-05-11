#ifndef SLAMGRAPH_VISUAL_H
#define SLAMGRAPH_VISUAL_H

#include <scavislam_messages/SLAMGraph.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Axes;
class BillboardLine;
class PointCloud;
}

namespace scavislam_rviz
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of SLAMGraphVisual represents the visualization of a single
// scavislam_messages::SLAMGraph message.
class SLAMGraphVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  SLAMGraphVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~SLAMGraphVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const scavislam_messages::SLAMGraph::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way SLAMGraphVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the SLAMGraph message.
  void setEdgeColor( Ogre::ColourValue &edgecolor,
                 float a );
  void setPointColor( Ogre::ColourValue &pointcolor,
                 float a );

  void setPointStyle( rviz::PointCloud::RenderMode m);
  void setPointScale( float s );
  void setVertexScale( float s );
  void setEdgeWidth( float w );
  void setShowPointConnections(bool c);
  void setShowPoints(bool c);

private:
  // The object implementing the actual arrow shape
  std::vector<boost::shared_ptr<rviz::Axes> > vertices_;
  boost::shared_ptr<rviz::PointCloud> points_;
  Ogre::ColourValue point_color_;
  std::vector<boost::shared_ptr<rviz::BillboardLine> > graph_edges_;
  std::vector<boost::shared_ptr<rviz::BillboardLine> > point_edges_;
  Ogre::ColourValue edge_color_;

  float alpha_;

  float vertex_scale_;
  float edge_width_;

  bool show_point_connections_;
  bool show_points_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the SLAMGraph message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

} // end namespace scavislam_rviz

#endif // SLAMGRAPH_VISUAL_H
