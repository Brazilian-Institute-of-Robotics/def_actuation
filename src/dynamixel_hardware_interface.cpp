#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include "control_table.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <string>
#include <sstream>

class MyRobot : public hardware_interface::RobotHW
{
  public:
    MyRobot(ros::NodeHandle nh);
    void initializeMotors();
    void readJointStates();
    bool switchTorque(int value);
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

    // motor configurations
    std::vector<double> *joint_offsets, *joint_velocity_limit;
    std::vector<int> *joint_goal_torque, *joint_op_mode;

    bool retrievedParams;
};

MyRobot::MyRobot(ros::NodeHandle nh)
    : gswPRO(NULL), gswMX(NULL), gsrPROvel(NULL), gsrMXvel(NULL), gbr(NULL), portHandler(NULL), packetHandler(NULL),
      joint_offsets(NULL), joint_goal_torque(NULL), joint_op_mode(NULL), joint_velocity_limit(NULL)
{
    // save if retrieving params worked
    retrievedParams = true;

    // get access inside the class to a nodeHandle
    node_handle_ = nh;
    pub_joint_states = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);

    // get motor configurations from parameter server
    joint_offsets = new std::vector<double>();
    joint_goal_torque = new std::vector<int>();
    joint_op_mode = new std::vector<int>();
    joint_velocity_limit = new std::vector<double>();

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        std::stringstream ss;
        ss << "/dyn_ef_robot/motor_configuration/joint" << i+1 << "/offset";
        double offset;
        if (node_handle_.getParam(ss.str(), offset))
        {
            joint_offsets->push_back(offset);
        }
        else
        {
            retrievedParams = false;
            ROS_INFO_STREAM("could not read parameter 'offset' from /dyn_ef_robot/motor_configuration for id " << MOTOR_IDS[i]);
        }

        int goal_torque;
        std::string torque_name;
        torque_name = IS_PRO[i] ? std::string("/goal_torque") : std::string("/goal_current");

        std::stringstream ss1;
        ss1 << "/dyn_ef_robot/motor_configuration/joint" << i+1 << torque_name;
        if (node_handle_.getParam(ss1.str(), goal_torque))
        {
            joint_goal_torque->push_back(goal_torque);
        }
        else
        {
            retrievedParams = false;
            ROS_INFO_STREAM("could not read parameter 'goal_torque' from /dyn_ef_robot/motor_configuration for id " << MOTOR_IDS[i]);
        }

        std::stringstream ss2;
        ss2 << "/dyn_ef_robot/motor_configuration/joint" << i+1 << "/operating_mode";
        int op_mode;
        if (node_handle_.getParam(ss2.str(), op_mode))
        {
            joint_op_mode->push_back(op_mode);
        }
        else
        {
            retrievedParams = false;
            ROS_INFO_STREAM("could not read parameter 'operating_mode' from /dyn_ef_robot/motor_configuration for id " << MOTOR_IDS[i]);
        }
        
        std::stringstream ss3;
        ss3 << "/dyn_ef_robot/motor_configuration/joint" << i+1 << "/velocity_limit";
        double vel_limit;
        if (node_handle_.getParam(ss3.str(), vel_limit))
        {
            joint_velocity_limit->push_back(vel_limit);
        }
        else
        {
            retrievedParams = false;
            ROS_INFO_STREAM("could not read parameter 'velocity_limit' from /dyn_ef_robot/motor_configuration for id " << MOTOR_IDS[i]);
        }
    }

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
                pos[i] = DXL_PRO_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_POSITION, 4) - (*joint_offsets)[i];
                vel[i] = DXL_PRO_VEL_RAW_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_PRO_PRESENT_VELOCITY, 4);
            }
        }
        else
        {
            if (dxl_getdata_result)
            {
                pos[i] = DXL_MX_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_POSITION, 4) - (*joint_offsets)[i];
                vel[i] = DXL_MX_VEL_RAW_TO_RAD * (int)gbr->getData(MOTOR_IDS[i], ADDR_MX_PRESENT_VELOCITY, 4);
            }
        }
        ROS_INFO_STREAM("ID " << i+1 << " pos: " << pos[i] << " vel: " << vel[i]);
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

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        int dxl_comm_result = 0;
        uint8_t dxl_error = 0;
        if (IS_PRO[i])
        {
            // set torque
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_PRO_GOAL_TORQUE, (*joint_goal_torque)[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting goal torque: " << dxl_comm_result);
                continue;
            }
            // set opmode
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_PRO_OP_MODE, (*joint_op_mode)[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting op mode: " << dxl_comm_result);
                continue;
            }
            // set velocity limit
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_PRO_VEL_LIMIT, (int)((*joint_velocity_limit)[i]/DXL_PRO_VEL_RAW_TO_RAD), &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting veloctiy limit: " << dxl_comm_result);
                continue;
            }
            // set p gain
            // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_PRO_VEL_P_GAIN, (*joint_op_mode)[i], &dxl_error);
            // if (dxl_comm_result != COMM_SUCCESS)
            // {
            //     ROS_WARN_STREAM("Error setting op mode: " << dxl_comm_result);
            //     continue;
            // }
        }
        else
        {
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_MX_GOAL_CURRENT, (*joint_goal_torque)[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting goal torque: " << dxl_comm_result);
                continue;
            }
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_MX_OP_MODE, (*joint_op_mode)[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting op mode: " << dxl_comm_result);
                continue;
            }
            // set velocity limit
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_MX_VEL_LIMIT, (int)((*joint_velocity_limit)[i]/DXL_MX_VEL_RAW_TO_RAD), &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0)
            {
                ROS_WARN_STREAM("Error setting veloctiy limit: " << dxl_comm_result);
                continue;
            }
        }
    }
    ROS_INFO("init finished");
}

bool MyRobot::switchTorque(int value)
{
    /* enables the torque of the specified dynamxiel motor 
    and returns true if it worked 
    */
    bool success = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (IS_PRO[i]) {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_PRO_TORQUE_ENABLE, value, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN_STREAM("dxl_comm_result: " << dxl_comm_result << " id is " << MOTOR_IDS[i]);
                success = false;
            }
            else if (dxl_error != 0)
            {
                ROS_WARN_STREAM("dxl_error: " << dxl_error << " id is " << MOTOR_IDS[i]);
                success = false;
            }
        } else {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_IDS[i], ADDR_MX_TORQUE_ENABLE, value, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                ROS_WARN_STREAM("dxl_comm_result: " << dxl_comm_result << " id is " << MOTOR_IDS[i]);
                success = false;
            }
            else if (dxl_error != 0)
            {
                ROS_WARN_STREAM("dxl_error: " << dxl_error << " id is " << MOTOR_IDS[i]);
                success = false;
            }
        }
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
        int val = (int)(cmd_raw[i][3] << 24 | cmd_raw[i][2] << 16 | cmd_raw[i][1] << 8 | cmd_raw[i][0]);
        ROS_INFO_STREAM("dxl id: " << i << " cmd: " << cmd[i] << ". vel_raw: " << val);
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

    ros::Time timestamp = ros::Time::now();
    ros::Rate rate(10); // in hz

    robot.switchTorque(TORQUE_ENABLE);

    while (ros::ok())
    {
        robot.readJointStates();

        robot.write();

        ros::Duration period = ros::Time::now() - timestamp;
        timestamp = ros::Time::now();
        cm.update(ros::Time::now(), period);

        // robot.write();
        rate.sleep();
    }
    std::cout << "Disabling Torque!" << std::endl;
    robot.switchTorque(TORQUE_DISABLE);
}
