int32 DIRECTION_UP = 0
int32 DIRECTION_RIGHT = 1
int32 DIRECTION_DOWN = 2
int32 DIRECTION_LEFT = 3

bool success
string error_reason
#surrounding_tiles are reported in order: left, forward, right
uint8[3] surrounding_tiles
int32 robot_dir