# def_actuation
Dynamic End-effector. Repository to implement actuation on arm manipulator.



NOTES:
- transmission_interface in .urdf and type of the controllers in def_controllers.yaml must be the same
- the parameters of the def_controllers.yaml must be loaded in the same namespace as the controllerManager in dynamixel_hardware_interface.cpp
- only one controllerManager can be running. We need the controllerManager in the .cpp so we cannot use the spawner in the .launch file