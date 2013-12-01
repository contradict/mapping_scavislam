#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include "slamgraph_visual.h"

#include "slamgraph_display.h"

namespace scavislam_rviz
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SLAMGraphDisplay::SLAMGraphDisplay()
{
  point_color_ = new rviz::ColorProperty( "Point Color", QColor( 204, 51, 204 ),
                                          "Color to draw the points.",
                                           this, SLOT( updateColorAndAlpha() ));

  point_style_property_ = new rviz::EnumProperty("Point Style", "Flat Squares",
                                                 "Rendering mode for points,",
                                                 this, SLOT( updatePointStyle() ));
  point_style_property_->addOption( "Points", rviz::PointCloud::RM_POINTS);
  point_style_property_->addOption( "Squares", rviz::PointCloud::RM_SQUARES);
  point_style_property_->addOption( "Flat Squares", rviz::PointCloud::RM_FLAT_SQUARES);
  point_style_property_->addOption( "Spheres", rviz::PointCloud::RM_SPHERES);
  point_style_property_->addOption( "Boxes", rviz::PointCloud::RM_BOXES);

  point_scale_property_ = new rviz::FloatProperty("Point Scale", 0.1,
                                                  "Scale of point visual.",
                                                  this, SLOT( updatePointScale() ));

  vertex_scale_property_ = new rviz::FloatProperty("Vertex Scale", 1.0,
                                                  "Scale of vertex visual.",
                                                  this, SLOT( updateVertexScale() ));

  edge_color_ = new rviz::ColorProperty( "Edge Color", QColor( 204, 51, 204 ),
                                         "Color to draw the graph edges.",
                                          this, SLOT( updateColorAndAlpha() ));

  edge_width_ = new rviz::FloatProperty( "Edge Width", 0.05,
                                        "Width of edge lines.",
                                        this, SLOT( updateEdgeWidth() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void SLAMGraphDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

SLAMGraphDisplay::~SLAMGraphDisplay()
{
}

// Clear the visuals by deleting their objects.
void SLAMGraphDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void SLAMGraphDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue edgecolor = edge_color_->getOgreColor();
  Ogre::ColourValue pointcolor = point_color_->getOgreColor();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setEdgeColor( edgecolor, alpha);
    visuals_[ i ]->setPointColor( pointcolor, alpha );
  }
}

void SLAMGraphDisplay::updatePointStyle()
{
  rviz::PointCloud::RenderMode mode =(rviz::PointCloud::RenderMode) point_style_property_->getOptionInt();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setPointStyle( mode );
  }
}

void SLAMGraphDisplay::updatePointScale()
{
  float scale = point_scale_property_->getFloat();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setPointScale( scale );
  }
}

void SLAMGraphDisplay::updateVertexScale()
{
  float scale = vertex_scale_property_->getFloat();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setVertexScale( scale );
  }
}

void SLAMGraphDisplay::updateEdgeWidth()
{
  float scale = edge_width_->getFloat();

  for( size_t i = 0; i < visuals_.size(); i++ )
  {
    visuals_[ i ]->setEdgeWidth( scale );
  }
}

// Set the number of past visuals to show.
void SLAMGraphDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
void SLAMGraphDisplay::processMessage( const scavislam_messages::SLAMGraph::ConstPtr& msg )
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this SLAMGraph message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<SLAMGraphVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new SLAMGraphVisual( context_->getSceneManager(), scene_node_ ));
  }

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue pointcolor = point_color_->getOgreColor();
  visual->setPointColor( pointcolor, alpha);
  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  float pscale = point_scale_property_->getFloat();
  visual->setPointScale(pscale);

  rviz::PointCloud::RenderMode mode =(rviz::PointCloud::RenderMode) point_style_property_->getOptionInt();
  visual->setPointStyle(mode);

  float vscale = vertex_scale_property_->getFloat();
  visual->setVertexScale(vscale);

  Ogre::ColourValue edgecolor = edge_color_->getOgreColor();
  visual->setEdgeColor( edgecolor, alpha );

  float width = edge_width_->getFloat();
  visual->setEdgeWidth(width);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace scavislam_rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(scavislam_rviz::SLAMGraphDisplay,rviz::Display )
