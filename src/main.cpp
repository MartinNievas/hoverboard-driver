#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dynamic_reconfigure/server.h>
#include "hoverboard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard_driver");

    Hoverboard& hoverboard = Hoverboard::getInstance();
    controller_manager::ControllerManager cm(&hoverboard);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();

    float loop_hz;
    ros::param::get("/robot/hardware_interface/loop_hz", loop_hz);
    ROS_INFO("main loop rate [Hz]: %f", loop_hz);

    ros::Rate rate(loop_hz);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        hoverboard.read();
        cm.update(time, period);
        hoverboard.write();
        hoverboard.tick();

        rate.sleep();
    }

    return 0;
}
