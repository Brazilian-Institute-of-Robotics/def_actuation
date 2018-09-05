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
    double pos[NUM_MOTORS]; // in rad
    double vel[NUM_MOTORS]; // in rad/s
    double eff[NUM_MOTORS];

    ros::NodeHandle node_handle_;

    // communication
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    // velocity sync write group for pro and mx
    dynamixel::GroupSyncWrite *gswPRO, *gswMX;
    // velocity sync read group for pro and mx
    dynamixel::GroupSyncRead *gsrPROvel, *gsrMXvel;
    // some helper variables
    uint8_t cmd_raw[NUM_MOTORS][4];
    int vel_raw[NUM_MOTORS];
    // bulk read for pos and vel
    dynamixel::GroupBulkRead *gbr;
};

MyRobot::MyRobot(ros::NodeHandle nh)
{
    // get access inside the class to a nodeHandle
    // node_handle_ = nh;

//    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
//    jnt_state_interface.registerHandle(state_handle_a);

//    registerInterface(&jnt_state_interface);

//    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("A"), &cmd[1]);
//    jnt_vel_interface.registerHandle(pos_handle_b);

//    registerInterface(&jnt_vel_interface);

    // // connect and register the joint state interface
    // std::vector<hardware_interface::JointStateHandle> state_handles;

    // for (int i = 0; i < NUM_MOTORS; i++)
    // {
    //     hardware_interface::JointStateHandle state_handle(JOINT_NAMES[i], &pos[0], &vel[0], &eff[0]);
    //     // jnt_state_interface.registerHandle( state_handle );
    //     state_handles.push_back(state_handle);
    //     jnt_state_interface.registerHandle( state_handles.back() );
    // }

    // registerInterface(&jnt_state_interface);

    // // connect and register the joint velocity interface
    // std::vector<hardware_interface::JointHandle> vel_handles;

    // for (int i = 0; i < NUM_MOTORS; i++)
    // {
    //     hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &cmd[0]);
    //     vel_handles.push_back(vel_handle);
    //     jnt_vel_interface.registerHandle( vel_handles.back() );
    // }

    // registerInterface(&jnt_vel_interface);
}

void MyRobot::readJointStates()
{
    // some helper variables
    bool dxl_comm_result = false;
    bool dxl_getdata_result_p = false;
    bool dxl_getdata_result_v = false;

    // do the actual read
    dxl_comm_result = gbr->txRxPacket();

    // check if parameters available, assign them to pos[] and vel[]
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (IS_PRO[i])
        {
            dxl_getdata_result_p = gbr->isAvailable(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 4);
            if (dxl_getdata_result_p != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead position failed for PRO", MOTOR_IDS[i]);
                continue;
            }
            dxl_getdata_result_v = gbr->isAvailable(MOTOR_IDS[i], ADDR_PRO_PRESENT_VELOCITY, 4);
            if (dxl_getdata_result_v != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead velocity failed for PRO", MOTOR_IDS[i]);
                continue;
            }
        }
        else
        {
            dxl_getdata_result_p = gbr->isAvailable(MOTOR_IDS[i], ADDR_MX_PRESENT_POSITION, 4);
            if (dxl_getdata_result_p != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead position failed for MX", MOTOR_IDS[i]);
                continue;
            }
            dxl_getdata_result_v = gbr->isAvailable(MOTOR_IDS[i], ADDR_MX_PRESENT_VELOCITY, 4);
            if (dxl_getdata_result_v != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead vellocity failed for MX", MOTOR_IDS[i]);
                continue;
            }
        }

        // get data and assign to pos[] and vel[]
        if (IS_PRO[i])
        {
            if (dxl_getdata_result_p)
            {
                pos[i] = DXL_PRO_TO_RAD * gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 4);
            }
            if (dxl_getdata_result_v)
            {
                vel[i] = DXL_PRO_VEL_RAW_TO_RAD * gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_VELOCITY, 4);
            }
        }
        else
        {
            if (dxl_getdata_result_p)
            {
                pos[i] = DXL_MX_TO_RAD * gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_POSITION, 4);
            }
            if (dxl_getdata_result_v)
            {
                vel[i] = DXL_MX_VEL_RAW_TO_RAD * gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_VELOCITY, 4);
            }
        }
    }
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

    // init bulk read stuff
    *gbr = dynamixel::GroupBulkRead(portHandler, packetHandler);

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

    // init bulk read conf
    bool dxl_addparam_result = false;
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (IS_PRO[i])
        {
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 4);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam pos failed for PRO", MOTOR_IDS[i]);
            }
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_PRO_PRESENT_VELOCITY, 4);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam vel failed for PRO", MOTOR_IDS[i]);
            }
        }
        else
        {
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_MX_PRESENT_POSITION, 4);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam pos failed for MX", MOTOR_IDS[i]);
            }
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_MX_PRESENT_VELOCITY, 4);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam vel failed for MX", MOTOR_IDS[i]);
            }
        }
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

void MyRobot::write()
{
    // TODO: implement safety limits, set velocity always to 0 if violated
    // get current cmd and convert to int
    bool dxl_addparam_result = false;
    bool dxl_comm_result = false;

    for (int i = 0; i < NUM_MOTORS; i++)
    {

        // get setpoint velocity from cmd-array, convert to int val
        if (IS_PRO[i])
        {
            vel_raw[i] = (int)(cmd[i] / DXL_PRO_VEL_RAW_TO_RAD);
        }
        else
        {
            vel_raw[i] = (int)(cmd[i] / DXL_MX_VEL_RAW_TO_RAD);
        }

        // Allocate goal position value into byte array
        cmd_raw[i][0] = DXL_LOBYTE(DXL_LOWORD(vel_raw[i]));
        cmd_raw[i][1] = DXL_HIBYTE(DXL_LOWORD(vel_raw[i]));
        cmd_raw[i][2] = DXL_LOBYTE(DXL_HIWORD(vel_raw[i]));
        cmd_raw[i][3] = DXL_HIBYTE(DXL_HIWORD(vel_raw[i]));

        // add to groupwrite  params
        if (IS_PRO[i])
        {
            dxl_addparam_result = gswPRO->addParam(MOTOR_IDS[i], cmd_raw[i]);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] groupSyncWrite addparam failed", MOTOR_IDS[i]);
                return;
            }
        }
        else
        {
            dxl_addparam_result = gswMX->addParam(MOTOR_IDS[i], cmd_raw[i]);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] groupSyncWrite addparam failed", MOTOR_IDS[i]);
                return;
            }
        }
    }

    // finally write commands
    dxl_comm_result = gswPRO->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_WARN("groupsyncwrite failed for PRO");
    dxl_comm_result = gswMX->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_WARN("groupsyncwrite failed for MX");

    // clear params for next round
    gswPRO->clearParam();
    gswMX->clearParam();
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

    robot.initializeMotors();

    controller_manager::ControllerManager cm(&robot, node_handle);

    ros::Rate rate(20); // 20 hz

    while (ros::ok())
    {
        robot.readJointStates();
        // cm.update(robot.get_time(), robot.get_period());
        rate.sleep();
    }
}
