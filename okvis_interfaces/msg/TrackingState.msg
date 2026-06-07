uint64 id # ID this tracking info refers to.
bool is_keyframe # Is it a keyframe?
bool is_lidar_keyframe # Is it a keyframe triggered by lidar?
uint8 tracking_quality # The tracking quality.
bool recognised_place # Has this fram recognised a place / relocalised / loop-closed?
bool is_full_graph_optimising # Is the background loop closure optimisation currently ongoing?
uint64 current_keyframe_id # The ID of the current keyframe.