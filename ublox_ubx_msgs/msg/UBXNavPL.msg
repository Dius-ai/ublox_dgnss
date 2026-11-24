# this message contains a UBX-NAV-PL (0x01 0x62) record protection level information
std_msgs/Header header # Header timestamp should be acquisition time

uint8 PL_FRAME_INVALID = 0
uint8 PL_FRAME_NED = 1              # north, east, down
uint8 PL_FRAME_LLV = 2              # longitudinal, lateral, vertical
uint8 PL_FRAME_HOV = 3              # horizontal semi-major, horizontal semi-minor, vertical

uint8 version               # message version
uint8 tmir_coeff            # TMIR (target misleading information risk) = tmir_coeff * 10^tmir_exp [%/epoch]
int8 tmir_exp
bool pl_pos_valid           # position protection level valid
uint8 pl_pos_frame          # PL_FRAME_* of pl_pos_1..3
bool pl_vel_valid           # velocity protection level valid
uint8 pl_vel_frame          # PL_FRAME_* of pl_vel_1..3
bool pl_time_valid          # time protection level valid
uint32 itow                 # ms - GPS Time of week of the navigation epoch
uint32 pl_pos_1             # mm - position protection level, 1st axis of pl_pos_frame
uint32 pl_pos_2             # mm - position protection level, 2nd axis
uint32 pl_pos_3             # mm - position protection level, 3rd axis
uint32 pl_vel_1             # mm/s - velocity protection level, 1st axis of pl_vel_frame
uint32 pl_vel_2             # mm/s - velocity protection level, 2nd axis
uint32 pl_vel_3             # mm/s - velocity protection level, 3rd axis
uint16 pl_pos_horiz_orient  # 1e-2 deg - orientation of the horizontal position frame
uint16 pl_vel_horiz_orient  # 1e-2 deg - orientation of the horizontal velocity frame
uint32 pl_time              # ns - time protection level
