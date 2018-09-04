#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "control_table.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

class MyRobot : public hardware_interface::RobotHW
{
  public:
    MyRobot();
    void initializeMotors();
    void readJointStates();
    bool enableTorque(uint8_t dynamxiel_id);

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    double cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

    dynamixel::PortHandler portHandler;
    dynamixel::PacketHandler packetHandler;
};

MyRobot::MyRobot()
{
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
    &portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    // Set the protocol version
    &packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

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

bool MyRobot::enableTorque(uint8_t dynamxiel_id)
{
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        packetHandler->printRxPacketError(dxl_error);
    }
    else
    {
        printf("Dynamixel has been successfully connected \n");
    }
}

// *********************************** main *********************************************
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_hardware_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MyRobot robot;
    controller_manager::ControllerManager cm(&robot, node_handle);

    ros::Rate rate(20); // 20 hz

    while (ros::ok())
    {
        // cm.update(robot.get_time(), robot.get_period());
        rate.sleep();
    }
}