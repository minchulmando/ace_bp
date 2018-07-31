# *******************************************************************
# This message was defined by Seokwon Kim (seokwonkim111@gmail.com)
# 2018.03.21 Initial version
# *******************************************************************

# time stamp (current algorithm time)
float32 time

# motion prediction x, y, s, n position (sn -> road map based)
Object_pred[] object_pred

# Object pose information
Object_xy object_xy
Object_sn object_sn

# Sementic information
uint16 id
uint16 tracking_id
uint16 tracking_stat
uint8 is_motion_prediction
uint8 is_sn_info
uint8 is_ego_lane
uint8 is_target_lane
uint8 is_target

int32[] road_segment_id
int32[] road_lane_id

#
uint8 object_class
float32 fusion_quality

# dimension of vehicle
float32 length
float32 width
float32 height

# tracking parameter
uint8 is_associated
uint16[] associating_measurement_id
float64[] mahal_dist
float32 existence_probability

# For mobileye
float32 range
float32 range_rate
uint8 valid

uint8 is_radar
uint8 is_lidar
uint8 is_mobileye
uint8 is_velodyne
