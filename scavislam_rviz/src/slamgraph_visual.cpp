
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
    // Here we create a node to store the pose of the graph's header frame
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

    vertices_.resize(msg->vertices.size());
    i=0;
    for(const auto vertex : msg->vertices){
        boost::shared_ptr<rviz::Axes> &vert = vertices_.at(i++);
        if(vert.get() == 0){
            vert.reset(new rviz::Axes(scene_manager_, frame_node_));
        }
        Ogre::Vector3 pos(vertex.pose.position.x,
                vertex.pose.position.y,
                vertex.pose.position.z);
        vert->setPosition(pos);
        Ogre::Quaternion orient(vertex.pose.orientation.w,
                vertex.pose.orientation.x,
                vertex.pose.orientation.y,
                vertex.pose.orientation.z);
        vert->setOrientation(orient);
    }

    std::vector<boost::shared_ptr<rviz::BillboardLine> > newgraphedges;
    std::vector<boost::shared_ptr<rviz::BillboardLine> > newpointedges;
    for(const auto vertex : msg->vertices){
        Ogre::Vector3 p0(vertex.pose.position.x,
                vertex.pose.position.y,
                vertex.pose.position.z);
        if(show_point_connections_){
            for(const auto point : vertex.points){
                boost::shared_ptr<rviz::BillboardLine> eln(new rviz::BillboardLine( scene_manager_, frame_node_ ));
                Ogre::Vector3 p1(newpoints[point.data].position);
                eln->addPoint(p0);
                eln->addPoint(p1);
                eln->setColor(point_color_.r, point_color_.g, point_color_.b, alpha_);
                eln->setLineWidth( edge_width_/2.0 );
                newpointedges.push_back(eln);
            }
        }
        for(const auto nb : vertex.neighbors){
            boost::shared_ptr<rviz::BillboardLine> eln(new rviz::BillboardLine( scene_manager_, frame_node_ ));
            Ogre::Vector3 p1(msg->vertices.at(nb.data).pose.position.x,
                    msg->vertices.at(nb.data).pose.position.y,
                    msg->vertices.at(nb.data).pose.position.z);
            eln->addPoint(p0);
            eln->addPoint(p1);
            eln->setColor(edge_color_.r, edge_color_.g, edge_color_.b, alpha_);
            eln->setLineWidth( edge_width_ );
            newgraphedges.push_back(eln);
        }
    }
    graph_edges_.swap(newgraphedges);
    point_edges_.swap(newpointedges);

    delete[] newpoints;
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
void SLAMGraphVisual::setEdgeColor(Ogre::ColourValue &edgecolor,
                               float a )
{
  edge_color_ = edgecolor;
  for(auto edge : graph_edges_)
      edge->setColor(edgecolor.r, edgecolor.g, edgecolor.b, a);
  alpha_ = a;
}

void SLAMGraphVisual::setPointColor( Ogre::ColourValue &pointcolor,
                                     float a )
{
  point_color_ = pointcolor;
  for(auto edge : point_edges_)
      edge->setColor(pointcolor.r, pointcolor.g, pointcolor.b, a);
  points_->setAlpha(a);
}

void SLAMGraphVisual::setPointStyle( rviz::PointCloud::RenderMode m)
{
    points_->setRenderMode(m);
}

void SLAMGraphVisual::setPointScale( float s )
{
    points_->setDimensions(s, s, s);
}

void SLAMGraphVisual::setVertexScale( float s )
{
    vertex_scale_ = s;
    for(auto vertex : vertices_){
        vertex->setScale(Ogre::Vector3(s,s,s));
    }
}

void SLAMGraphVisual::setEdgeWidth(float w)
{
    edge_width_ = w;
    for(auto edge : graph_edges_){
        edge->setLineWidth(w);
    }
}

void SLAMGraphVisual::setShowPointConnections(bool c)
{
    show_point_connections_ = c;
}

} // end namespace rviz_plugin_tutorials

