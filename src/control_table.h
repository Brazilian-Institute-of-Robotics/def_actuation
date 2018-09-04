
// trajectory start pont tolerance in rad
#define TRAJECTORY_START_TOLERANCE 0.1 // approx. 6 deg

// Control table address
#define ADDR_PRO_OP_MODE 11  // 1 byte
#define ADDR_PRO_TORQUE_ENABLE        562 // 1 byte
#define ADDR_PRO_GOAL_POSITION        596 // 4 byte 
#define ADDR_PRO_PRESENT_POSITION     611 // 4 byte
#define ADDR_PRO_GOAL_VELOCITY        600 // 4 byte
#define ADDR_PRO_PRESENT_VELOCITY     615 // 4 byte
#define ADDR_PRO_GOAL_TORQUE          604 // 2 byte
#define DXL_PRO_TO_RAD                0.00001251822551901305 // factor to multiply to get rad value from raw position
#define DXL_PRO_VEL_RAW_TO_RAD        0.012518241414906177
#define DXL_PRO_GAIN                  20

// Firmware 2.0, check control table
// http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
#define ADDR_MX_OP_MODE              11   // 1 byte
#define ADDR_MX_MAXPOS_LIMIT         48   // 4 byte
#define ADDR_MX_MINPOS_LIMIT         52   // 4 byte
#define ADDR_MX_TORQUE_ENABLE        64   // 1 byte
#define ADDR_MX_GOAL_POSITION        116  // 4 byte 
#define ADDR_MX_PRESENT_POSITION     132  // 4 byte
#define ADDR_MX_GOAL_VELOCITY        104  // 4 byte
#define ADDR_MX_PRESENT_VELOCITY     128  // 4 byte
#define ADDR_MX_VEL_LIMIT            44   // 4 byte
#define DXL_MX_VEL_RAW_TO_RAD        1.4388494353441252 //0.229*2*math.pi
#define DXL_MX_TO_RAD                0.001533981 // factor to multiplicate to get rad value from raw position
#define DXL_MX_POS_OFFSET            2047
#define DXL_MX_GAIN                  20

// Protocol version
#define PROTOCOL_VERSION             2

#define BAUDRATE                     1000000
#define DEVICENAME                   "/dev/ttyUSB0"

#define PRO_OPMODE_VEL  1
#define PRO_OPMODE_POS  3
#define TORQUE_ENABLE                1                             // Value for enabling the torque
#define TORQUE_DISABLE               0                             // Value for disabling the torque
#define DXL_PRO_MINIMUM_POSITION_VALUE   -150000                       // Dynamixel will rotate between this value
#define DXL_PRO_MAXIMUM_POSITION_VALUE   150000                        // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_PRO_MOVING_STATUS_THRESHOLD  20                            // Dynamixel moving status threshold

#define COMM_SUCCESS                 0                             // Communication Success result value
#define COMM_TX_FAIL                 -1001                         // Communication Tx Failed