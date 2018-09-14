# def_actuation
Implementation of a hardware interface for every type of Dynamixel actuators using protocol 2.0.
The hardware interface currently only supports velocity control

# Installation
Clone all packages from the dynamic end effector repositiories and use ```rosdep``` to install other dependecies.


# How to use?

IMPORTANT:
- Transmission_interface in .urdf and type of the controllers in def_controllers.yaml must be of the same type (e.g. ```hardware_interface/VelocityJointInterface```)
- The parameters of the ```def_controllers.yaml``` must be loaded in the same namespace as the ```controllerManager``` in ```dynamixel_hardware_interface.cpp```
- Only one ```controllerManager``` can be running. In the current implementation the ```controllerManager``` has to be ran in the .cpp so the spawner in the .launch file cannot be used

# Configure it!

The paramters for the Dynamixel actuators are stored in ```config/dxl_motor_configuration.yaml```. Currently only ```offset, goal_torque, operating_mode, velocity_limit``` are used in ```dynamixel_hardware_interface.cpp```.
```def_controllers.yaml``` holds the parameters for the joint trajectory velocity controller such as PID gain for each joint. Handle with care!

Configure the name, type and ID of Dynamixel motors to use in ```src/control_table.h```. Also leave register adresses there if you are implementing new functionalities
for the Dynamixel motors. Remeber to also change in ```config/dxl_motor_configuration.yaml``` if necessary.

# Launch it!

To start the controller simply launch ```dynamixel_hardware_control.launch```.

For validation purposes it might be necessary to have a look at the robots current position, but not enable torque (e.g. to move the arm by hand and see if the arm is calibrated correctly).
To do this, launch ```display.launch``` (you need the package ```dynamic_end_effector``` which contains the robot description as .urdf)

For combined use with ```def_cam_teledyne_nano``` just launch ```dyn_ef_robot_bringup_hardware.launch```.