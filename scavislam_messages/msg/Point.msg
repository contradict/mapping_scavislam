geometry_msgs/Vector3 xyz_anchor     # actual 3d position
uint32                anchorframe_id # id of the anchor frame tom which the
                                     # point is anchored
geometry_msgs/Vector3 anchor_obs_pyr # initial observation in anchor frame
uint32                anchor_level   # pyramid level of observation
geometry_msgs/Vector3 normal_anchor  # estimate of surface normal
uint32[]              vis_set        # set of frame vertex ids from which
                                     # the point is visible
