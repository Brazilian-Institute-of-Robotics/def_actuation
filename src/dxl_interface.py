#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Interface for trajectory following for dynamixel pro

import os, sys, time, rospy, math
import tty, termios
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
import actionlib

# Messages
import std_msgs
import control_msgs.msg as ctrl_msgs
#from control_msgs.msg import FollowJointTrajectoryAction
import trajectory_msgs.msg as traj_msgs
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

# soft speed limit in rad/s
SOFT_SPEED_LIMIT = 1.0

# trajectory start pont tolerance in rad
TRAJECTORY_START_TOLERANCE = 0.1 # approx. 6 deg

# Control table address
ADDR_PRO_OP_MODE             = 11  # 1 byte
ADDR_PRO_TORQUE_ENABLE       = 562 # 1 byte                         # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 596 # 4 byte 
ADDR_PRO_PRESENT_POSITION    = 611 # 4 byte
ADDR_PRO_GOAL_VELOCITY       = 600 # 4 byte
ADDR_PRO_PRESENT_VELOCITY    = 615 # 4 byte
ADDR_PRO_GOAL_TORQUE         = 604 # 2 byte
ADDR_PRO_PRESENT_CURRENT     = 621 # 2 byte
DXL_PRO_TO_RAD               = 0.00001251822551901305 # factor to multiply to get rad value from raw position
DXL_PRO_VEL_RAW_TO_RAD       = 0.012518241414906177
DXL_PRO_GAIN                 = 20

# Firmware 2.0, check control table
# http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106(2.0).htm
ADDR_106_OP_MODE             = 11   # 1 byte
ADDR_106_MAXPOS_LIMIT        = 48   # 4 byte
ADDR_106_MINPOS_LIMIT        = 52   # 4 byte
ADDR_106_TORQUE_ENABLE       = 64   # 1 byte
ADDR_106_GOAL_POSITION       = 116  # 4 byte 
ADDR_106_PRESENT_POSITION    = 132  # 4 byte
ADDR_106_GOAL_VELOCITY       = 104  # 4 byte
ADDR_106_PRESENT_VELOCITY    = 128  # 4 byte
ADDR_106_VEL_LIMIT           = 44   # 4 byte
DXL_106_VEL_RAW_TO_RAD       = 0.229*2*math.pi
DXL_106_TO_RAD               = 0.001533981 # factor to multiplicate to get rad value from raw position
DXL_106_POS_OFFSET           = 2047
DXL_106_GAIN                 = 20

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

DXL_PRO_ID                  = 2
DXL_106_ID                  = 1
BAUDRATE                    = 57600
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')

PRO_OPMODE_VEL = 1
PRO_OPMODE_POS = 3
TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_PRO_MINIMUM_POSITION_VALUE  = -150000                       # Dynamixel will rotate between this value
DXL_PRO_MAXIMUM_POSITION_VALUE  = 150000                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_PRO_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)



index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result

dxl_error = 0                                               # Dynamixel error
dxl_present_position = 0                                    # Present position

class DxlInterface:
    # # Joint names and Dynamixel IDs
    # joints = {
    #     "base_link_to_base_yaw_link_joint": 1,
    #     "base_yaw_link_to_first_link_joint": 2,
    #     "first_link_to_second_link_joint": 3,
    #     "second_link_to_third_link_joint": 4,
    #     "third_link_to_fourth_link_joint": 5,
    #     "fourth_link_to_fifth_link_joint": 6
    # }

    # # Decode which joints are PRO
    # joint_pro = {
    #     "base_link_to_base_yaw_link_joint": True,
    #     "base_yaw_link_to_first_link_joint": True,
    #     "first_link_to_second_link_joint": True,
    #     "second_link_to_third_link_joint": True,
    #     "third_link_to_fourth_link_joint": False,
    #     "fourth_link_to_fifth_link_joint": False
    # }

    # Joint names and Dynamixel IDs
    joints = {
        "base_link_to_base_yaw_link_joint": (1, False),
        "fourth_link_to_fifth_link_joint": (10, False)
    }

    # Decode which joints are PRO
    joint_pro = {
        "base_link_to_base_yaw_link_joint": True,
        "fourth_link_to_fifth_link_joint": False
    }

    currentTrajectory = JointTrajectory()
    trajectoryStartTime = 0.0
    currentIndex = 0
    currentTrajectoryLen = 0
    moving = False
    goalReached = True
    _trajFeedback = ctrl_msgs.FollowJointTrajectoryActionFeedback()
    _trajResult = ctrl_msgs.FollowJointTrajectoryActionResult()

    groupread_num = dynamixel.groupBulkRead(port_num, PROTOCOL_VERSION)
    groupwrite_num = dynamixel.groupBulkWrite(port_num, PROTOCOL_VERSION)

    def __init__(self):
        # Initialize PacketHandler Structs
        dynamixel.packetHandler()
        # Open port
        if dynamixel.openPort(port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            quit()

        # Set port baudrate
        if dynamixel.setBaudRate(port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            quit()

        for jn in g.goal.trajectory.joint_names:

            if joints[jn][1]:
                # set to read bytes containing position, velocity and current
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(groupread_num, joints[jn][0], ADDR_PRO_PRESENT_POSITION, 12)).value
                if dxl_addparam_result != 1:
                    print("addparam pro not suck-sessful")
            else:
                # set to read bytes containing position, velocity and current
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(groupread_num, joints[jn][0], ADDR_106_PRESENT_CURRENT, 10)).value
                if dxl_addparam_result != 1:
                    print("addparam mx not suck-sessful")


    def initDxl106(self):
        # set opmode to velocity control
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_OP_MODE, PRO_OPMODE_POS)
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_MAXPOS_LIMIT, 4090)
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_MINPOS_LIMIT, 10)
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_VEL_LIMIT, 500)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_ENABLE)

    def initDxlPro(self):
        # set opmode to velocity control
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_OP_MODE, PRO_OPMODE_POS)

        # Enable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        resulttx = dynamixel.getTxRxResult(port_num, PROTOCOL_VERSION)
        if resulttx != COMM_SUCCESS:
            pass
        elif dynamixel.getRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getRxPacketError(port_num, PROTOCOL_VERSION))
        else:
            print("Dynamixel has been successfully connected")



    # Set new Trajectory as goal
    def setNewTrajectory(self,g):
        # if another trajectory is in progress, decline
        if self.moving:
            return

        # check if requested joints match the existent
        if not set(g.goal.trajectory.joint_names) <= set(self.joints.keys()):
            return
        
        # check if initial position is close enough to the trajectory initial position
        currentState = self.getJointState()
        for jn in g.goal.trajectory.joint_names:
            # get index in trajectory for reading position for joint
            currentIndex = g.goal.trajectory.joint_names.index(jn)

            # get index in current joint state
            currentJsIdx = currentState.name.index(jn)
            actualPos = currentState.position[currentJsIdx]
            if (g.goal.trajectory.points[0].positions[currentIndex] < actualPos - TRAJECTORY_START_TOLERANCE) or (g.goal.trajectory.points[0].positions[currentIndex] > actualPos + TRAJECTORY_START_TOLERANCE):
                # start position violates limits, abort
                rospy.loginfo("Trajectory start position violates Limits, aborting. Joint: %s. Actual: %f. Trajectory: %f"%(jn,actualPos,g.goal.trajectory.points[0].positions[currentIndex]))
                # todo: some actionlib result-stuff
                return

        # if execution got here, everything is fine and the trajectory can be executed
        self.currentTrajectory = g.goal.trajectory
        self.currentTrajectoryLen = len(self.currentTrajectory.points)
        self.moving = True
        self.goalReached = False
        self.currentIndex = 0
        self.trajectoryStartTime = rospy.Time.now()

            


    # gets raw position and velocity data from given dynamixels and returns
    # constructed sensor_msgs.JointState for further use in ROS
    def getJointState(self):

        # construct joint state
        js_msg = JointState()

        # construct header
        js_msg.header = std_msgs.msg.Header()
        js_msg.header.stamp = rospy.Time.now()

        dynamixel.groupBulkReadTxRxPacket(groupread_num)

        for js in self.joints:
            js_msg.name.append(js)
            if joints[js][1]:
                dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(groupread_num, joints[js][0], ADDR_PRO_PRESENT_POSITION, 12)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupBulkRead PRO getdata failed" % (joints[js][0]))
                    continue
                raw = dynamixel.groupBulkReadGetData(groupread_num, joints[js][0], ADDR_PRO_PRESENT_POSITION, 4)
                js_msg.position.append(raw*DXL_PRO_TO_RAD)
                raw = dynamixel.groupBulkReadGetData(groupread_num, joints[js][0], ADDR_PRO_PRESENT_VELOCITY, 4)
                js_msg.velocity.append(raw*DXL_PRO_VEL_RAW_TO_RAD)

            else:
                dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(groupread_num, joints[js][0], ADDR_106_PRESENT_POSITION, 10)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupBulkRead MX getdata failed" % (joints[js][0]))
                    continue
                raw = dynamixel.groupBulkReadGetData(groupread_num, joints[js][0], ADDR_106_PRESENT_POSITION, 4)
                js_msg.position.append(raw*DXL_106_TO_RAD)
                raw = dynamixel.groupBulkReadGetData(groupread_num, joints[js][0], ADDR_106_PRESENT_VELOCITY, 4)
                js_msg.velocity.append(raw*DXL_106_VEL_RAW_TO_RAD)

        
        return js_msg


    
    def move(self):
        """move joints to follow given trajectory, or keep current position"""
        currentState = self.getJointState()
        trajectoryExecTime = rospy.Time.now() - self.trajectoryStartTime

        # compute indice of trajectory that fits best for current time
        if self.currentIndex < self.currentTrajectoryLen - 2:
            if self.currentTrajectory.points[self.currentIndex + 1].time_from_start < trajectoryExecTime:
                self.currentIndex += 1
        else:
            # reached goal
            # todo: maybe keep position if dynamixels will turn from weight
            # send velocity to dynamixel
#            for jn in self.currentTrajectory.joint_names:
#                if self.joints[jn][1]:
#                    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_GOAL_POSITION, 0)
#                else:
#                    dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_GOAL_POSITION, 0)

            self.moving = False
            self.goalReached = True
            return
        
        for jn in self.currentTrajectory.joint_names:
            # get index in trajectory for reading position for joint
            jointIndex = self.currentTrajectory.joint_names.index(jn)

            # compute actual position
            currentJsIdx = currentState.name.index(jn)
            actualPos = currentState.position[currentJsIdx]

            # compute setpoint position
            firstPos = (self.currentTrajectory.points[self.currentIndex].positions[jointIndex],self.currentTrajectory.points[self.currentIndex].time_from_start)
            secPos = (self.currentTrajectory.points[self.currentIndex+1].positions[jointIndex],self.currentTrajectory.points[self.currentIndex+1].time_from_start)
            # print(firstPos)
            # print(secPos)
            posDiff = secPos[0] - firstPos[0]
            timeDiff = secPos[1] - firstPos[1]
            # print("posDiff=%f"%posDiff)
            timeFromLastPoint = trajectoryExecTime - self.currentTrajectory.points[self.currentIndex].time_from_start
            # print(timeFromLastPoint)
            # posError = actualPos - (firstPos[0] + (timeFromLastPoint/timeDiff)*posDiff)
            posError = (firstPos[0] + (timeFromLastPoint/timeDiff)*posDiff)
            # print("pos error: %f" % posError)

            # send velocity to dynamixel
            if self.joints[jn][1]:
                print("sending %i to dynamixel pro" % int(posError/DXL_PRO_TO_RAD))
                dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, self.joints[jn][0], ADDR_PRO_GOAL_POSITION, int(posError/DXL_PRO_TO_RAD))
            else:
                print("sending %i to dynamixel 106" % int(posError/DXL_106_TO_RAD))
                dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, self.joints[jn][0], ADDR_106_GOAL_POSITION, int(posError/DXL_106_TO_RAD))


    def test(self):
        g = ctrl_msgs.FollowJointTrajectoryActionGoal()
        g.goal.trajectory.header.stamp = rospy.Time.now()
        g.goal.trajectory.joint_names = ["base_link_to_base_yaw_link_joint", "fourth_link_to_fifth_link_joint"]
        msg = traj_msgs.JointTrajectoryPoint()
        msg.time_from_start = rospy.Duration.from_sec(0.0)
        msg.positions = [0.0, 0.0]
        g.goal.trajectory.points.append(msg)
        msg = traj_msgs.JointTrajectoryPoint()
        msg.time_from_start = rospy.Duration.from_sec(2.0)
        msg.positions = [1.57, 6.0]
        g.goal.trajectory.points.append(msg)
        msg = traj_msgs.JointTrajectoryPoint()
        msg.time_from_start = rospy.Duration.from_sec(4.0)
        msg.positions = [0, 2.0]
        g.goal.trajectory.points.append(msg)
        msg = traj_msgs.JointTrajectoryPoint()
        msg.time_from_start = rospy.Duration.from_sec(6.0)
        msg.positions = [-1.57, -1.0]
        g.goal.trajectory.points.append(msg)
        msg = traj_msgs.JointTrajectoryPoint()
        msg.time_from_start = rospy.Duration.from_sec(8.0)
        msg.positions = [0.0, -2.0]
        g.goal.trajectory.points.append(msg)


        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_DISABLE)

        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_OP_MODE, PRO_OPMODE_POS)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_OP_MODE, PRO_OPMODE_POS)

        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_ENABLE)

        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_GOAL_POSITION, 0)
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_GOAL_POSITION, 2047)
    
        time.sleep(3)


        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_DISABLE)

        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_OP_MODE, PRO_OPMODE_VEL)
        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_OP_MODE, PRO_OPMODE_VEL)


        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        # dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_ENABLE)

        self.setNewTrajectory(g)





    def start(self):

        actual_vel = 0
        upgoing = True
        rospy.init_node('dynamixelpronode', anonymous=True)

        # joint state publisher
        jointStatePub = rospy.Publisher('joint_states', JointState, queue_size=10)

        _server = actionlib.SimpleActionServer('follow_dxl_trajectory', ctrl_msgs.FollowJointTrajectoryAction, self.setNewTrajectory, False)
        _server.start()
        #server.set_succeeded()
        #rospy.Subscriber("/dyn_ef_robot/dyn_ef_robot_controller/follow_joint_trajectory/goal", String, setNewTrajectory)
        rate = rospy.Rate(20) # 10hz


        self.test()

        while not rospy.is_shutdown():
            # print("Press any key to continue! (or press ESC to quit!)")
            # if getch() == chr(ESC_ASCII_VALUE):
            #     break
            # print("hallo")

            if upgoing:
                actual_vel += 800
                if actual_vel >= 4000:
                    upgoing = False
            else:
                actual_vel -= 800
                if actual_vel <= -4000:
                    upgoing = True

            # Write goal position
            # dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_GOAL_VELOCITY, actual_vel)
            # dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_GOAL_VELOCITY, actual_vel/16)

            currentJointState = self.getJointState()
            jointStatePub.publish(currentJointState)

            if not self.goalReached:
                self.move()
            # if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            #     print("not successful")
            #     dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
            # elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            #     dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

            # while 1:
            #     # Read present position
            #     print("read present position")
            #     dxl_present_position = dynamixel.read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION)
            #     if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            #         dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
            #     elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            #         dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

            #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

            #     if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            #         break

            # Change goal position
            # if index == 0:
            #     index = 1
            # else:
            #     index = 0
            
            rate.sleep()

        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_106_ID, ADDR_106_TORQUE_ENABLE, TORQUE_DISABLE)
        # Close port
        dynamixel.closePort(port_num)



    

if __name__ == '__main__':
    try:
        dxl = DxlInterface()
        dxl.initDxl106()
        dxl.initDxlPro()
        dxl.start()
    except rospy.ROSInterruptException:
        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_PRO_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
