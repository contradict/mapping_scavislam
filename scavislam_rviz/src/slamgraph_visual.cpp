
#include <unordered_map>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <scavislam_rviz/slamgraph_visual.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#undef HAVE_OPENNI

#include <scavislam_ros/stereograph.h>

#include <eigen_conversions/eigen_msg.h>

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
    /*
    SE3d T_c1_from_w
          = GET_MAP_ELEM(id1,graph_draw_data->vertex_table).T_me_from_world;
      SE3d T_c2_from_w
          = GET_MAP_ELEM(id2,graph_draw_data->vertex_table).T_me_from_world;
      Draw3d::line(T_c1_from_w.inverse().translation(),
                   T_c2_from_w.inverse().translation());

                   //How do I color a vertex here?
        int pose_id_1 = it_win1->first;
        StereoGraph::WindowType wtype_1 = it_win1->second;
        if (wtype_1==StereoGraph::INNER)
            glColor3f(1,0,0);
        else
            glColor3f(0.5,0.5,0.5);
        const StereoGraph::Vertex & v
            = GET_MAP_ELEM(pose_id_1, graph_draw_data->vertex_table);
        Draw3d::pose(v.T_me_from_world.inverse());


    */
   /* build window table */
    std::unordered_map<int, int> doublewindow;
    pcl::PointCloud<WindowTablePoint> wtps;
    fromROSMsg( msg->doublewindow, wtps );
    for( const auto& pt : wtps ){
        doublewindow.insert(std::pair<int,int>(pt.vertex_id, pt.window));
    }

    /* extract camera extrinsics */
    Eigen::Affine3d T_base_from_camera;
    tf::poseMsgToEigen(msg->T_base_from_camera, T_base_from_camera);

    /* build vertex table while drawing vertices */
    std::unordered_map<int, Eigen::Affine3d> vertex_table;
    if( vertices_.size() < msg->vertices.size() )
        vertices_.resize(msg->vertices.size());
    int i=0;
    for(const auto vertex : msg->vertices){
        boost::shared_ptr<rviz::Axes> &vert = vertices_.at(i++);
        if( vert.get() == 0 )
            vert.reset(new rviz::Axes(scene_manager_, frame_node_));
        Eigen::Affine3d vpose;
        tf::poseMsgToEigen( vertex.pose, vpose );
        vpose = T_base_from_camera*vpose.inverse();
        Eigen::Vector3d t=vpose.translation();
        Ogre::Vector3 pos(t(0),
                          t(1),
                          t(2));
        vert->setPosition(pos);
        Eigen::Quaterniond q(vpose.rotation());
        Ogre::Quaternion orient(q.w(),
                                q.x(),
                                q.y(),
                                q.z());
        vert->setOrientation(orient);
        vert->setToDefaultColors();
        if( doublewindow[vertex.own_id] == 1 ) {
            Ogre::ColourValue c=vert->getDefaultXColor();
            c.a*=0.5;
            vert->setXColor(c);
            c=vert->getDefaultYColor();
            c.a*=0.5;
            vert->setYColor(c);
            c=vert->getDefaultZColor();
            c.a*=0.5;
            vert->setZColor(c);
        }
        vertex_table.insert(std::pair<int, Eigen::Affine3d>(vertex.own_id, vpose));
    }
    for( int j=i; j<vertices_.size(); j++ ) {
        Ogre::ColourValue off( 0.0f, 0.0f, 0.0f, 0.0f );
        vertices_[j]->setXColor( off );
        vertices_[j]->setYColor( off );
        vertices_[j]->setZColor( off );
    }

    /* Use transforms from vertex table to draw points */
    rviz::PointCloud::Point *newpoints = new rviz::PointCloud::Point[msg->points.size()];
    std::vector<boost::shared_ptr<rviz::BillboardLine> > newpointedges;
    i=0;
    for(const auto point :msg->points) {
        newpoints[i].color = point_color_;
        Eigen::Vector3d ap;
        tf::vectorMsgToEigen(point.xyz_anchor, ap);
        Eigen::Affine3d T_world_from_anchorframe=vertex_table[ point.anchorframe_id ];
        Eigen::Vector3d wp=T_world_from_anchorframe*ap;
        newpoints[i].position[0]=wp(0);
        newpoints[i].position[1]=wp(1);
        newpoints[i].position[2]=wp(2);
        if( show_point_connections_ ) {
            newpointedges.push_back(
                    boost::shared_ptr<rviz::BillboardLine>(
                        new rviz::BillboardLine( scene_manager_, frame_node_ )
                        )
                    );
            Ogre::Vector3 p0( T_world_from_anchorframe.translation()(0),
                    T_world_from_anchorframe.translation()(1),
                    T_world_from_anchorframe.translation()(2));
            newpointedges.back()->addPoint(p0);
            newpointedges.back()->addPoint(newpoints[i].position);
        }
        i++;
    }
    points_->clear();
    points_->addPoints(newpoints, i);
    point_edges_.swap(newpointedges);
    delete[] newpoints;

    std::vector<boost::shared_ptr<rviz::BillboardLine> > newgraphedges;
    for(const auto edge : msg->edges){
        Eigen::Affine3d T_world_from_1=vertex_table[ edge.id1 ];
        Ogre::Vector3 p0( T_world_from_1.translation()(0),
                          T_world_from_1.translation()(1),
                          T_world_from_1.translation()(2));
        Eigen::Affine3d T_world_from_2=vertex_table[ edge.id2 ];
        Ogre::Vector3 p1( T_world_from_2.translation()(0),
                          T_world_from_2.translation()(1),
                          T_world_from_2.translation()(2));
        double alpha = std::min(1.0, edge.strength/100.0);
        newgraphedges.push_back(
                boost::shared_ptr<rviz::BillboardLine>(
                    new rviz::BillboardLine( scene_manager_, frame_node_ )
                    )
                );
        newgraphedges.back()->addPoint(p0);
        newgraphedges.back()->addPoint(p1);
        newgraphedges.back()->setColor(edge.color.r,
                                       edge.color.g,
                                       edge.color.b,
                                       alpha);
    }
    graph_edges_.swap(newgraphedges);
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

