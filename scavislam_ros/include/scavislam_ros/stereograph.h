#ifndef __STEROEGRAPH_H__
#define __STEROEGRAPH_H__
#include <backend.h>
#include <data_structures.h>
#undef HAVE_OPENNI
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

#include <scavislam_messages/SLAMGraph.h>

struct WindowTablePoint {
    uint32_t vertex_id;
    uint32_t window;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (WindowTablePoint,
                                   (uint32_t, vertex_id, vertex_id)
                                   (uint32_t, window, window)
        )

struct NeighborIdsPoint {
    uint32_t strength;
    uint32_t vertex_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (NeighborIdsPoint,
                                   (uint32_t, strength, strength)
                                   (uint32_t, vertex_id, vertex_id)
        )

struct FeatureTablePoint {
    PCL_ADD_POINT4D;
    uint32_t level;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (FeatureTablePoint,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, level, level)
        )


namespace scavislam_ros {

void StereoGraphToMessage(const ScaViSLAM::StereoGraph& graph,
                          const Sophus::SE3d& T_base_from_camera,
                          scavislam_messages::SLAMGraph *msg);

void MessageToStereoGraph(const scavislam_messages::SLAMGraphPtr msg,
                          ScaViSLAM::StereoGraph *graph);

void DrawDataToMessage(const ScaViSLAM::BackendDrawDataPtr graph_draw_data,
                       const Sophus::SE3d& T_base_from_camera,
                       scavislam_messages::SLAMGraph *msg);

void NeighborhoodToMessage(const ScaViSLAM::NeighborhoodPtr neighborhood,
                           const Sophus::SE3d& T_base_from_camera,
                           scavislam_messages::SLAMGraph *msg);
}

#endif // __STEROEGRAPH_H__
