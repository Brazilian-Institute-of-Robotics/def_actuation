#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/LoadControllerRequest.h>
#include <controller_manager_msgs/LoadControllerResponse.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_controller_loader");
    ros::NodeHandle node_handle;

    // service msg to load controller
    controller_manager_msgs::LoadController srv_load;
    srv_load.request.name = "dyn_ef_robot_controller";

    // service msg to start controller
    // ros::ServiceClient start_controller_client = node_handle.serviceClient<controller_manager_msgs::SwitchController>("/dyn_ef_robot/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_switch;
    srv_switch.request.start_controllers.push_back("dyn_ef_robot_controller");
    srv_switch.request.strictness = 1;

    // flags if services are already called
    bool success_start_ctrl = false;
    bool success_load_ctrl = false;

    ros::Rate rate(5); // in hz
    while (ros::ok())
    {
        // try to load dyn_ef_robot_controller
        if (ros::service::exists("/dyn_ef_robot/controller_manager/load_controller", true))
        {
            if (!success_load_ctrl)
            {
                success_load_ctrl = ros::service::call("/dyn_ef_robot/controller_manager/load_controller", srv_load);
                ROS_INFO("Loading controllers %s", success_load_ctrl ? "succeeded" : "FAILED");
            }
        }

        // try to start dyn_ef_robot_controller
        if (ros::service::exists("/dyn_ef_robot/controller_manager/switch_controller", true))
        {
            if (!success_start_ctrl)
            {
                success_start_ctrl = ros::service::call("/dyn_ef_robot/controller_manager/switch_controller", srv_switch);
                ROS_INFO("Starting controllers %s", success_start_ctrl ? "succeeded" : "FAILED");
            }
        }

        // shutdown node if all services are successfully calleddddd
        if (success_start_ctrl && success_load_ctrl)
        {
            ros::shutdown();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
