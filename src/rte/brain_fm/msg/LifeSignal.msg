#######################################
# Header
# time stamp is important information to determine this module is alive or not
Header header

#######################################
# moudle id
#  0: ego position estimation
#  1: map_finder
#  2: local route
#  3: object fusion
#  4: behaivor planning
#  5: local planning
#  6: lateral control
#  7: longitudianl control
# 10: map matching
# 20: traffic light recognition
uint32 ID_POSE_ESTIMATION = 1 
uint32 ID_MAP_FINDER = 2
uint32 ID_LOCAL_ROUTE = 3
uint32 ID_OBJECT_FUSION = 4
uint32 ID_BEHAVIOR_PLANNING = 5
uint32 ID_LOCAL_PLANNING = 6
uint32 ID_LON_CONTROL = 7
uint32 ID_LAT_CONTROL = 8
uint32 ID_LANE_FUSION = 9
uint32 ID_MAP_MATCHING = 10
uint32 ID_TLR = 11
uint32 id

#######################################
# module name
string module

#######################################
# module status
bool is_fault

#######################################
# error code
#   0x01: Device error
#   0x02: Function error
#   0x04: Task error
uint8 error_code



