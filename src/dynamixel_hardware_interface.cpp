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
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    ros::NodeHandle node_handle_;
    
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    // velocity sync write group for pro and mx
    dynamixel::GroupSyncWrite *gswPRO, *gswMX;
    // position sync read group for pro and mx
    dynamixel::GroupSyncRead *gsrPROpos, *gsrMXpos;
};

MyRobot::MyRobot(ros::NodeHandle nh)
{
    node_handle_ = nh;
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("base_link_to_base_yaw_link_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("base_yaw_link_to_first_link_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("base_link_to_base_yaw_link_joint"), &cmd[0]);
    jnt_vel_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("base_yaw_link_to_first_link_joint"), &cmd[1]);
    jnt_vel_interface.registerHandle(pos_handle_b);

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

}

// *********************************** main *********************************************
int main(int argc, char **argv)
{
    ros::NodeHandle nh("~");
    MyRobot robot(nh);
    ros::init(argc, argv, "dynamixel_hardware_interface");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(20); // 20 hz

    while (ros::ok())
    {
        // cm.update(robot.get_time(), robot.get_period());
        rate.sleep();
    }
}