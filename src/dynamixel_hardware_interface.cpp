#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include "control_table.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>

class MyRobot : public hardware_interface::RobotHW
{
  public:
    MyRobot(ros::NodeHandle nh);
    void initializeMotors();
    void readJointStates();
    bool enableTorque(uint8_t dynamixel_id, uint16_t addr_torque_enable);
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
    ros::Publisher pub_joint_states;

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
    : gswPRO(NULL), gswMX(NULL), gsrPROvel(NULL), gsrMXvel(NULL), gbr(NULL), portHandler(NULL), packetHandler(NULL)
{
    // get access inside the class to a nodeHandle
    node_handle_ = nh;
    pub_joint_states = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);

    // // connect and register the joint state interface
    std::vector<hardware_interface::JointStateHandle> state_handles;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        hardware_interface::JointStateHandle state_handle(JOINT_NAMES[i], &pos[i], &vel[i], &eff[i]);
        // jnt_state_interface.registerHandle( state_handle );
        state_handles.push_back(state_handle);
        jnt_state_interface.registerHandle(state_handles.back());
    }

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    std::vector<hardware_interface::JointHandle> vel_handles;

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(JOINT_NAMES[i]), &cmd[i]);
        vel_handles.push_back(vel_handle);
        jnt_vel_interface.registerHandle(vel_handles.back());
    }

    registerInterface(&jnt_vel_interface);
}

void MyRobot::readJointStates()
{
    // some helper variables
    bool dxl_comm_result = false;
    bool dxl_getdata_result = false;

    // some joint_state msg to publish the joint states
    sensor_msgs::JointState *joint_state_msg = new sensor_msgs::JointState;
    static int seq = 0;
    joint_state_msg->header.seq = seq;
    seq += 1;
    joint_state_msg->header.frame_id = ""; //should probably fill with world
    joint_state_msg->header.stamp = ros::Time::now();

    // do the actual read
    dxl_comm_result = gbr->txRxPacket();

    
    // check if parameters available, assign them to pos[] and vel[]
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (IS_PRO[i])
        {
            // make sure
            dxl_getdata_result = gbr->isAvailable(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 12);
            if (dxl_getdata_result != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead position failed for PRO", MOTOR_IDS[i]);
                continue;
            }
        }
        else
        {
            dxl_getdata_result = gbr->isAvailable(MOTOR_IDS[i], ADDR_MX_PRESENT_CURRENT, 10);
            if (dxl_getdata_result != true)
            {
                ROS_WARN("[ID:%03d] groupBulkRead position failed for MX", MOTOR_IDS[i]);
                continue;
            }
        }

        // get data and assign to pos[] and vel[]
        if (IS_PRO[i])
        {
            if (dxl_getdata_result)
            {
                pos[i] = DXL_PRO_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 4);
                vel[i] = DXL_PRO_VEL_RAW_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_VELOCITY, 4);
            }
        }
        else
        {
            if (dxl_getdata_result)
            {
                pos[i] = DXL_MX_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_POSITION, 4);
                vel[i] = DXL_MX_VEL_RAW_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_VELOCITY, 4);
            }
        }
        joint_state_msg->name.push_back(JOINT_NAMES[i]);
        joint_state_msg->position.push_back(pos[i]);
        joint_state_msg->velocity.push_back(vel[i]);
        joint_state_msg->effort.push_back(0.0);
    }
    pub_joint_states.publish(*joint_state_msg);
}

void MyRobot::initializeMotors()
{
    // Set the port path
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    // Set the protocol version
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Goal position length is 4 for both MX and PRO
    gswPRO = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, 4);
    gswMX = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, 4);
    gbr = new dynamixel::GroupBulkRead(portHandler, packetHandler);

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
            // start reading at present position, next 12 bytes are pos[4], vel[4], (2 undefined bytes) and current[2]
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 12);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam pos failed for PRO", MOTOR_IDS[i]);
            }
        }
        else
        {
            // start reading at present current, next 10 bytes are current[2], vel[4], pos[4]
            dxl_addparam_result = gbr->addParam(MOTOR_IDS[i], ADDR_MX_PRESENT_CURRENT, 10);
            if (dxl_addparam_result != true)
            {
                ROS_WARN("[ID:%03d] grouBulkRead addparam pos failed for MX", MOTOR_IDS[i]);
            }
        }
    }

    // TODO configure motors: pos-, vel-, torque-limits and opmode
    
    ROS_INFO("init finished");
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
    ros::NodeHandle node_handle("dyn_ef_robot");
    MyRobot robot(node_handle);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // open communication and read initial position
    robot.initializeMotors();

    // nodeHandle in ControllerManager must be the same namespace as in def_controllers.yaml!
    controller_manager::ControllerManager cm(&robot, node_handle);

    // loading all the controller as only one controller manager can be used
    // bool success_loading_def_controller = cm.loadController("def_controller");
    // ROS_INFO("Loading controller 'def_controller' %s", success_loading_def_controller ? "Succeeded" : "FAILED");
    // starting the loaded controllers
    // if (success_loading_def_controller)
    // {
    //     std::vector<std::string> start_controller, stop_controller;
    //     start_controller.push_back("def_controller");
    //     stop_controller.push_back("");
    //     bool success_started_def_controller = cm.switchController(start_controller, stop_controller, 1);
    //     ROS_INFO("Starting controller 'def_controller' %s", success_loading_def_controller ? "Succeeded" : "FAILED");
    // }

    ros::Time timestamp = ros::Time::now();
    ros::Rate rate(10); // in hz

    while (ros::ok())
    {
        robot.readJointStates();

        ros::Duration period = ros::Time::now() - timestamp;
        timestamp = ros::Time::now();
        cm.update(ros::Time::now(), period);

        // robot.write();
        rate.sleep();
    }
}
