
// trajectory start pont tolerance in rad
#define TRAJECTORY_START_TOLERANCE 0.1 // approx. 6 deg

// Control table address
#define ADDR_PRO_OP_MODE 11  // 1 byte
#define ADDR_PRO_TORQUE_ENABLE        562 // 1 byte
#define ADDR_PRO_GOAL_POSITION        596 // 4 byte 
#define ADDR_PRO_PRESENT_POSITION     611 // 4 byte
#define ADDR_PRO_GOAL_VELOCITY        600 // 4 byte
#define ADDR_PRO_PRESENT_VELOCITY     615 // 4 byte
#define ADDR_PRO_PRESENT_CURRENT      621  // 2 byte
#define ADDR_PRO_GOAL_TORQUE          604 // 2 byte
#define ADDR_PRO_VEL_I_GAIN           586 // 2 byte
#define ADDR_PRO_VEL_P_GAIN           588 // 2 byte
#define ADDR_PRO_VEL_LIMIT            32 // 4 byte
#define DXL_PRO_TO_RAD                0.00001251822551901305 // factor to multiply to get rad value from raw position
#define DXL_PRO_VEL_RAW_TO_RAD        0.00020863735691510296 // (0.00199234/60)*2*pi
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
#define ADDR_MX_GOAL_CURRENT         102  // 2 byte
#define ADDR_MX_PRESENT_VELOCITY     128  // 4 byte
#define ADDR_MX_PRESENT_CURRENT      126  // 2 byte
#define ADDR_MX_VEL_LIMIT            44   // 4 byte
#define ADDR_MX_VEL_I_GAIN           76 // 2 byte
#define ADDR_MX_VEL_P_GAIN           78 // 2 byte
#define DXL_MX_VEL_RAW_TO_RAD        0.023980823922402087 // 2*math.pi*0.229/60
#define DXL_MX_TO_RAD                0.001533981 // factor to multiply to get rad value from raw position
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

#define NUM_MOTORS 6
const uint8_t MOTOR_IDS[NUM_MOTORS] = {1,2,3,4,5,6};
const std::string JOINT_NAMES[NUM_MOTORS] = {
    "base_link_to_base_yaw_link_joint",
    "base_yaw_link_to_first_link_joint",
    "first_link_to_second_link_joint",
    "second_link_to_third_link_joint",
    "third_link_to_fourth_link_joint",
    "fourth_link_to_fifth_link_joint",
};
const bool IS_PRO[NUM_MOTORS] = {true,true,true,true,false,false};
