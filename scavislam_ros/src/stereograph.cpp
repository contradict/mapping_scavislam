#include <scavislam_ros/stereograph.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <visiontools/accessor_macros.h>


namespace scavislam_ros {

void poseToMessage(const Sophus::SE3d pose,
                   geometry_msgs::Pose *p)
{
    tf::quaternionEigenToMsg(pose.so3().unit_quaternion(),
            p->orientation);
    p->position.x = pose.translation()(0);
    p->position.y = pose.translation()(1);
    p->position.z = pose.translation()(2);
}

int edgeTableToMessage(const ScaViSLAM::StereoGraph::EdgeTable& edge_table,
                       std::vector<scavislam_messages::Edge>::iterator edges,
                       const Eigen::Vector4d force_color,
                       ScaViSLAM::StereoGraph::WindowTable* const double_window,
                       bool is_new)
{
    int ii = 0;
    for( auto& pr : edge_table ) {
        if( (double_window != NULL) &&
                ( VisionTools::IS_IN_SET(pr.second->vertex_id1,*double_window)==false
                  || VisionTools::IS_IN_SET(pr.second->vertex_id2,*double_window)==false))
            continue;
        edges->id1 = pr.second->vertex_id1;
        edges->id2 = pr.second->vertex_id2;
        if(force_color == Eigen::Vector4d(0,0,0,0)) {
            if (!pr.second->is_marginalized()) {
                edges->color.r = 0.75;
                edges->color.g = 0.0;
                edges->color.b = 0.0;
                edges->color.a = 0.5;
            } else {
                edges->color.r = 0.75;
                edges->color.g = 0.75;
                edges->color.b = 0.75;
                edges->color.a = 0.5;
            }
            if (pr.second->error>0.0000001) {
                edges->color.r = 0.0;
                edges->color.g = 0.0;
                edges->color.b = 1.0;
                edges->color.a = 0.75;
            }
        } else {
            edges->color.r = force_color(0);
            edges->color.g = force_color(1);
            edges->color.b = force_color(2);
            edges->color.a = force_color(3);
        }


        poseToMessage( pr.second->T_1_from_2,
                       &edges->T_1_from_2);

        edges->strength = pr.second->strength;
        edges->error = pr.second->error;
        edges->edge_type = pr.second->edge_type;
        edges->is_new = is_new;

        tf::matrixEigenToMsg(pr.second->Lambda_2_from_1, edges->Lambda_2_from_1);
        tf::matrixEigenToMsg(pr.second->Lambda_1_from_2, edges->Lambda_1_from_2);
        edges++;
    }
    return ii;
}

void windowTableToMessage(const ScaViSLAM::StereoGraph::WindowTable& double_window,
                          sensor_msgs::PointCloud2* doublewindow)
{
    /* Copy window table */
    pcl::PointCloud<WindowTablePoint> mwt;
    mwt.resize(double_window.size());
    int ii=0;
    for( auto& pr : double_window ){
        mwt.points[ii].vertex_id = pr.first;
        mwt.points[ii++].window = pr.second;
    }
    pcl::toROSMsg(mwt, *doublewindow);
}


int vertexTableToMessage(const ScaViSLAM::StereoGraph::VertexTable& vertex_table,
                         std::vector<scavislam_messages::Vertex>::iterator vertices
                         )
{
    int ii=0;
    for( auto& pr : vertex_table ) {

        vertices->own_id = pr.first;

        pcl::PointCloud<NeighborIdsPoint> mnt;
        mnt.resize(pr.second->neighbor_ids_ordered_by_strength.size());
        int jj=0;
        for( auto& npr : pr.second->neighbor_ids_ordered_by_strength){
            mnt.points[jj].strength = npr.first;
            mnt.points[jj++].vertex_id = npr.second;
        }
        pcl::toROSMsg(mnt, vertices->neighbor_ids_ordered_by_strength);

        vertices->fixed = pr.second->fixed;

        /*
        pcl::PointCloud<FeatureTablePoint> mft;
        mft.resize(pr.second->feature_table.size());
        jj=0;
        for( auto& npr : pr.second->feature_table ) {
            mft.points[jj].x = npr.second.center(0);
            mft.points[jj].y = npr.second.center(1);
            mft.points[jj].z = npr.second.center(2);
            mft.points[jj++].level = npr.second.level;
        }
        pcl::toROSMsg(mft, vertices->feature_table);
        */

        poseToMessage(pr.second->T_me_from_world,
                      &vertices->pose);

        vertices++;
        ii++;
    }
    return ii;
}

int pointTableToMessage( const ScaViSLAM::StereoGraph::PointTable& point_table,
                         std::vector<scavislam_messages::Point>::iterator points)
{
    int ii=0;
    for( auto& pr : point_table ) {
        tf::vectorEigenToMsg(pr.second->xyz_anchor, points->xyz_anchor);

        for(auto& i : pr.second->vis_set)
            points->vis_set.push_back(i);

        points->anchorframe_id = pr.second->anchorframe_id;

        tf::vectorEigenToMsg(pr.second->anchor_obs_pyr, points->anchor_obs_pyr);

        points->anchor_level = pr.second->anchor_level;

        tf::vectorEigenToMsg(pr.second->normal_anchor, points->normal_anchor);

        points++;
        ii++;
    }
    return ii;
}

void DrawDataToMessage(const ScaViSLAM::BackendDrawDataPtr graph_draw_data,
                       const Sophus::SE3d& T_base_from_camera,
                       scavislam_messages::SLAMGraph *msg)
{

    poseToMessage(T_base_from_camera,
                  &msg->T_base_from_camera);

    /*---------- edge table ---------------*/
    msg->edges.resize(graph_draw_data->edge_table.size() + graph_draw_data->new_edges.size());
    int ii = edgeTableToMessage( graph_draw_data->edge_table,
                                 msg->edges.begin(),
                                 Eigen::Vector4d(0,0,0,0),
                                 &graph_draw_data->double_window,
                                 false
                               );
    /*---------- new edges ---------------*/
    edgeTableToMessage( graph_draw_data->new_edges,
                        msg->edges.begin() + ii,
                        Eigen::Vector4d(0.75, 0, 0, 0.),
                        &graph_draw_data->double_window,
                        true
                      );
    /*-------------------------*/

    /*----------- double window --------------*/
    windowTableToMessage( graph_draw_data->double_window,
                          &msg->doublewindow );
    /*-------------------------*/

    /* active point set */
    for( auto i : graph_draw_data->active_point_set )
        msg->active_points.push_back(i);

    /*-------------------------*/
    msg->vertices.resize(graph_draw_data->vertex_table.size());
    vertexTableToMessage( graph_draw_data->vertex_table,
                          msg->vertices.begin()
                        );
    /*-------------------------*/
    msg->points.resize(graph_draw_data->point_table.size());
    pointTableToMessage( graph_draw_data->point_table,
                         msg->points.begin() );
}

void StereoGraphToMessage(const ScaViSLAM::StereoGraph& graph,
                          const Sophus::SE3d& T_base_from_camera,
                          scavislam_messages::SLAMGraph *msg)
{

    poseToMessage(T_base_from_camera,
                  &msg->T_base_from_camera);

    windowTableToMessage( graph.double_window(),
                         &msg->doublewindow);


    /* active point set */
    for( auto i : graph.active_point_set() )
        msg->active_points.push_back(i);

    /* outer point set */
    for( auto i : graph.outer_point_set() )
        msg->outer_points.push_back(i);

    /* vertex table */
    msg->vertices.resize(graph.vertex_table().size());
    vertexTableToMessage( graph.vertex_table(),
                          msg->vertices.begin() );

    /* point table */
    msg->points.resize(graph.point_table().size());
    pointTableToMessage( graph.point_table(),
                         msg->points.begin() );

    msg->edges.resize(graph.edge_table().size());
    edgeTableToMessage( graph.edge_table(),
                        msg->edges.begin(),
                        Eigen::Vector4d(0,0,0,0),
                        NULL,
                        false);
}

void MessageToStereoGraph(const scavislam_messages::SLAMGraphPtr msg,
                          ScaViSLAM::StereoGraph *graph)
{
}

void NeighborhoodToMessage(const ScaViSLAM::NeighborhoodPtr neighborhood,
                           const Sophus::SE3d& T_base_from_camera,
                           scavislam_messages::SLAMGraph *msg)
{
    poseToMessage(T_base_from_camera,
                  &msg->T_base_from_camera);

    msg->vertices.resize(neighborhood->vertex_map.size());
    int ii=0;
    for ( auto& pr : neighborhood->vertex_map ) {
        msg->vertices[ii].own_id = pr.first;
        //Sophus::SE3d T1 = T_base_from_camera*(pr.second.T_me_from_w.inverse());
        poseToMessage(pr.second.T_me_from_w,
                      &msg->vertices[ii].pose);

        pcl::PointCloud<NeighborIdsPoint> mnt;
        mnt.resize(pr.second.strength_to_neighbors.size());
        int jj=0;
        for( auto& npr : pr.second.strength_to_neighbors){
            mnt.points[jj].strength = npr.first;
            mnt.points[jj++].vertex_id = npr.second;
            scavislam_messages::Edge edge;
            edge.id1 = pr.first;
            edge.id2 = npr.second;
            edge.strength = npr.first;
            edge.is_new = false;
            msg->edges.push_back(edge);
        }
        pcl::toROSMsg(mnt, msg->vertices[ii].neighbor_ids_ordered_by_strength);

        msg->vertices[ii].fixed = false;

        pcl::PointCloud<FeatureTablePoint> mft;
        mft.resize(pr.second.feat_map.size());
        jj=0;
        for( auto& npr : pr.second.feat_map ) {
            mft.points[jj].x = npr.second.center(0);
            mft.points[jj].y = npr.second.center(1);
            mft.points[jj].z = npr.second.center(2);
            mft.points[jj++].level = npr.second.level;
        }
        pcl::toROSMsg(mft, msg->vertices[ii].feature_table);

        ii++;
    }

    msg->points.resize(neighborhood->point_list.size());
    ii = 0;
    for( auto ap : neighborhood->point_list ) {
        //Eigen::Vector3d pt(T_base_from_camera*(VisionTools::GET_MAP_ELEM(ap->anchor_id, neighborhood->vertex_map).T_me_from_w.inverse())
        //        *ap->xyz_anchor);
        tf::vectorEigenToMsg(ap->xyz_anchor, msg->points[ii].xyz_anchor);
        msg->points[ii].anchorframe_id = ap->anchor_id;
        tf::vectorEigenToMsg(ap->anchor_obs_pyr, msg->points[ii].anchor_obs_pyr);
        msg->points[ii].anchor_level = ap->anchor_level;
        tf::vectorEigenToMsg(ap->normal_anchor, msg->points[ii].normal_anchor);
        ii++;
    }

    // send empty window table to avoid empty fields list on receipt
    pcl::PointCloud<WindowTablePoint> mwt;
    mwt.resize(0);
    pcl::toROSMsg(mwt, msg->doublewindow);
}

}
