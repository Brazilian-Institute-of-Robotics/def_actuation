#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "control_table.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

class MyRobot : public hardware_interface::RobotHW
{
  public:
    MyRobot(ros::NodeHandle nh);
    void initializeMotors();
    void readJointStates();
    bool enableTorque(uint8_t dynamxiel_id, uint16_t addr_torque_enable);
    bool disableTorque(uint8_t dynamixel_id, uint16_t addr_torque_enable);
    void write();

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmd[NUM_MOTORS];
    double pos[NUM_MOTORS];
    double vel[NUM_MOTORS];
    double eff[NUM_MOTORS];

    ros::NodeHandle node_handle_;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    // velocity sync write group for pro and mx
    dynamixel::GroupSyncWrite *gswPRO, *gswMX;
    // some helper variables
    uint8_t cmd_raw[NUM_MOTORS][4];
    int vel_raw[NUM_MOTORS];
    // position sync read group for pro and mx
    dynamixel::GroupSyncRead *gsrPROpos, *gsrMXpos;
};

MyRobot::MyRobot(ros::NodeHandle nh)
{
    // get access inside the class to a nodeHandle
    node_handle_ = nh;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle *state_handles[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        *state_handles[i] = hardware_interface::JointStateHandle(JOINT_NAMES[i], &pos[0], &vel[0], &eff[0]);
        jnt_state_interface.registerHandle(*state_handles[i]);
    }

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle *vel_handles[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        *vel_handles[i] = hardware_interface::JointHandle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &cmd[0]);
        jnt_vel_interface.registerHandle(*vel_handles[i]);
    }

    registerInterface(&jnt_vel_interface);
}

void MyRobot::readJointStates()
{
}

void MyRobot::initializeMotors()
{
    // Set the port path
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    // Set the protocol version
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Goal position length is 4 for both MX and PRO
    *gswPRO = dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, 4);
    *gswMX = dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_VELOCITY, 4);
    // Present position, length also identical
    *gsrPROpos = dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, 4);
    *gsrMXpos = dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, 4);

    // Open port
    if (portHandler->openPort())
    {
        ROS_INFO("Succeeded to open the port!\n");
    }
    else
    {
        ROS_WARN("Failed to open the port!\n");
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        ROS_INFO("Succeeded to change the baudrate!\n");
    }
    else
    {
        ROS_WARN("Failed to change the baudrate!\n");
    }
}

bool MyRobot::enableTorque(uint8_t dynamixel_id, uint16_t addr_torque_enable)
{
    /* tries to enable the torque of the specified dynamxiel motor 
    and returns true if it worked 
    */
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, addr_torque_enable, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN_STREAM("dxl_comm_result: " << dxl_comm_result);
        return false;
    }
    else if (dxl_error != 0)
    {
        ROS_WARN_STREAM("dxl_error: " << dxl_error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Torque of dynamixel with id " << dynamixel_id << "successfully enabled");
        return true;
    }
}

bool MyRobot::disableTorque(uint8_t dynamixel_id, uint16_t addr_torque_enable)
{
    /* tries to disable the torque of the specified dynamxiel motor 
    and returns true if it worked 
    */
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, addr_torque_enable, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN_STREAM("dxl_comm_result: " << dxl_comm_result);
        return false;
    }
    else if (dxl_error != 0)
    {
        ROS_WARN_STREAM("dxl_error: " << dxl_error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Torque of dynamixel with id " << dynamixel_id << "successfully disabled");
        return true;
    }
}


void MyRobot::write() {
    // get current cmd and convert to int
    //for (int i = 0; i < NUM_MOTORS; i++)

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (IS_PRO[i]) {
            vel_raw[i] = (int)(cmd[i]/DXL_PRO_VEL_RAW_TO_RAD);
        } else {
            vel_raw[i] = (int)(cmd[i]/DXL_MX_VEL_RAW_TO_RAD);
        }
        // Allocate goal position value into byte array
        cmd_raw[i][0] = DXL_LOBYTE(DXL_LOWORD(vel_raw[i]));
        cmd_raw[i][1] = DXL_HIBYTE(DXL_LOWORD(vel_raw[i]));
        cmd_raw[i][2] = DXL_LOBYTE(DXL_HIWORD(vel_raw[i]));
        cmd_raw[i][3] = DXL_HIBYTE(DXL_HIWORD(vel_raw[i]));
    }


// *********************************** main *********************************************
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_hardware_interface");
    ros::NodeHandle node_handle;
    ros::NodeHandle nh("~");
    MyRobot robot(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    controller_manager::ControllerManager cm(&robot, node_handle);

    ros::Rate rate(20); // 20 hz

    while (ros::ok())
    {
        robot.readJointStates();
        // cm.update(robot.get_time(), robot.get_period());
        rate.sleep();
    }
}