#include "config.h"
#include "hoverboard.h"
#include "HoverboardAPI.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <dynamic_reconfigure/server.h>

int port_fd = -1;

int serialWrite(unsigned char *data, int len) {
    return (int)write(port_fd, data, len);
}

void readCallback(PROTOCOL_STAT* s, PARAMSTAT* param, uint8_t fn_type, unsigned char* content, int len) {
  if (fn_type == FN_TYPE_POST_READRESPONSE) {
    if (param->code == HoverboardAPI::Codes::sensHall) {
      Hoverboard::getInstance().hallCallback();
    } else if (param->code == HoverboardAPI::Codes::sensElectrical) {
      Hoverboard::getInstance().electricalCallback();
    }
  }
}

Hoverboard& Hoverboard::getInstance() {
  static Hoverboard hoverboard;
  return hoverboard;
}

Hoverboard::Hoverboard() {
  hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel",
      &joints[0].pos.data,
      &joints[0].vel.data,
      &joints[0].eff.data);
  hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel",
      &joints[1].pos.data,
      &joints[1].vel.data,
      &joints[1].eff.data);
  joint_state_interface.registerHandle (left_wheel_state_handle);
  joint_state_interface.registerHandle (right_wheel_state_handle);
  registerInterface(&joint_state_interface);

  hardware_interface::JointHandle left_wheel_vel_handle(joint_state_interface.getHandle("left_wheel"),
      &joints[0].cmd.data);
  hardware_interface::JointHandle right_wheel_vel_handle(joint_state_interface.getHandle("right_wheel"),
      &joints[1].cmd.data);
  velocity_joint_interface.registerHandle (left_wheel_vel_handle);
  velocity_joint_interface.registerHandle (right_wheel_vel_handle);
  registerInterface(&velocity_joint_interface);

    // These publishers are only for debugging purposes
    left_pos_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/position", 3);
    right_pos_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/position", 3);
    left_vel_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/velocity", 3);
    right_vel_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/velocity", 3);
    left_eff_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/eff", 3);
    right_eff_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/eff", 3);
    left_cmd_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/cmd", 3);
    right_cmd_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/cmd", 3);
    left_cur_pub  = nh.advertise<std_msgs::Float64>("hoverboard/left_wheel/current", 3);
    right_cur_pub = nh.advertise<std_msgs::Float64>("hoverboard/right_wheel/current", 3);
    voltage_pub   = nh.advertise<std_msgs::Float64>("hoverboard/battery_voltage", 3);

    // nav_msgs/Odometry
    odom_test  = nh.advertise<nav_msgs::Odometry>("hoverboard/odom", 3);

    ros::param::get("/robot/wheel_radius", _wheel_radius);
    ROS_INFO("wheel radius: %f", _wheel_radius);

    ros::param::get("/robot/distance_betwen_wheels", _length_between_wheels);
    ROS_INFO("wheel separation: %f", _length_between_wheels);

    ros::param::get("/robot/serial_port", _serial_port);
    ROS_INFO("serial port: %s", _serial_port.c_str());

    ros::param::get("/robot/direction_correction", _direction_correction);
    ROS_INFO("Wheel direction: %f", _direction_correction);

    if ((port_fd = open(_serial_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL("Cannot open serial port to hoverboard");
        exit(-1);
    }

    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);

    api = new HoverboardAPI(serialWrite);
    api->scheduleRead(HoverboardAPI::Codes::sensHall, -1, 50, PROTOCOL_SOM_NOACK);
    api->scheduleRead(HoverboardAPI::Codes::sensElectrical, -1, 50, PROTOCOL_SOM_NOACK);

    // Support dynamic reconfigure
    dsrv = new dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<hoverboard_driver::HoverboardConfig>::CallbackType cb =
      boost::bind(&Hoverboard::reconfigure_callback, this, _1, _2);
    dsrv->setCallback(cb);

    api->updateParamHandler(HoverboardAPI::Codes::sensHall, readCallback);
    api->updateParamHandler(HoverboardAPI::Codes::sensElectrical, readCallback);
    api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    robot_x = 0.0;
    robot_y = 0.0;
    robot_th = 0.0;

    robot_vx = 0.0;
    robot_vy = 0.0;
    robot_vth = 0.0;

}

Hoverboard::~Hoverboard() {
    if (port_fd != -1) 
        close(port_fd);
    delete api;
}

void Hoverboard::reconfigure_callback(hoverboard_driver::HoverboardConfig& _config, uint32_t level) {
  config = _config;
  have_config = true;

  printf("Reconfigured PID to [%d, %d, %d, %d]\n", config.Kp, config.Ki, config.Kd, config.Incr);
  api->sendPIDControl(config.Kp, config.Ki, config.Kd, config.Incr);
}

void Hoverboard::read() {
  if (port_fd != -1) {
    const int max_length = 1024; // HoverboardAPI limit
    unsigned char c;
    int i = 0, r = 0;

    while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
      api->protocolPush(c);

    if (i > 0)
      last_read = ros::Time::now();

    if (r < 0 && errno != EAGAIN)
      ROS_ERROR("Reading from serial %s failed: %d", _serial_port.c_str(), r);
  }

  if ((ros::Time::now() - last_read).toSec() > 1) {
    ROS_FATAL("Timeout reading from serial %s failed", _serial_port.c_str());
  }

  api->requestRead(HoverboardAPI::Codes::sensHall, PROTOCOL_SOM_NOACK);
  api->requestRead(HoverboardAPI::Codes::sensElectrical, PROTOCOL_SOM_NOACK);
}

void Hoverboard::hallCallback() {
    // Convert m/s to rad/s
    double sens_speed0 = api->getSpeed0_mms();
    double sens_speed1 = api->getSpeed1_mms();

    ros::param::get("/robot/direction_correction", _direction_correction);

    joints[0].vel.data = _direction_correction * (sens_speed0 / 1000.0);
    joints[1].vel.data = _direction_correction * (sens_speed1 / 1000.0);
    joints[0].pos.data = _direction_correction * (api->getPosition0_mm() / 1000.0);
    joints[1].pos.data = _direction_correction * (api->getPosition1_mm() / 1000.0);
    left_vel_pub.publish(joints[0].vel);
    right_vel_pub.publish(joints[1].vel);
    left_pos_pub.publish(joints[0].pos);
    right_pos_pub.publish(joints[1].pos);

    current_time = ros::Time::now();

    double v_right = joints[0].vel.data; // right_wheel speed in m/s
    double v_left = joints[1].vel.data; // left_wheel speed in m/s

    robot_vx = ((v_right + v_left) / 2);
    robot_vy = 0;
    robot_vth = ((v_right - v_left)/_length_between_wheels);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (robot_vx * cos(robot_th) - robot_vy * sin(robot_th)) * dt;
    double delta_y = (robot_vx * sin(robot_th) + robot_vy * cos(robot_th)) * dt;
    double delta_th = robot_vth * dt;

    robot_x += delta_x;
    robot_y += delta_y;
    robot_th += delta_th;

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = robot_x;
    odom.pose.pose.position.y = robot_y;
    odom.pose.pose.position.z = 0.0;

    // odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = robot_vx;
    odom.twist.twist.linear.y = robot_vy;
    odom.twist.twist.angular.z = robot_vth;

    //publish the message
    odom_test.publish(odom);

    last_time = current_time;

    // printf("[%.3f, %.3f] -> [%.3f, %.3f]\n", sens_speed0, sens_speed1, joints[0].vel.data, joints[1].vel.data);
}

void Hoverboard::electricalCallback() {
    std_msgs::Float64 f;

    ROS_INFO("electrical Callback");
    f.data = api->getMotorAmpsAvg(0);
    left_cur_pub.publish(f);

    f.data = api->getMotorAmpsAvg(1);
    right_cur_pub.publish(f);

    f.data = api->getBatteryVoltage();
    voltage_pub.publish(f);
}

void Hoverboard::write() {
    if (port_fd == -1) {
        ROS_ERROR("Attempt to write on closed serial");
        return;
    }
    // Inform interested parties about the commands we've got
    left_cmd_pub.publish(joints[0].cmd);
    right_cmd_pub.publish(joints[1].cmd);

#ifdef CONTROL_PWM
    int left_pwm  = joints[0].cmd.data * 30;
    int right_pwm = joints[1].cmd.data * 30;
    api->sendDifferentialPWM (left_pwm, right_pwm);
    ROS_INFO("PWM send: %d %d", left_pwm, right_pwm);
#else
    // Cap according to dynamic_reconfigure
    // Convert rad/s to m/s

    double left_speed = _direction_correction * joints[0].cmd.data * _wheel_radius;
    double right_speed = _direction_correction * joints[1].cmd.data * _wheel_radius;
    const int max_power = have_config ? config.MaxPwr : 100;
    const int min_speed = have_config ? config.MinSpd : 40;
    api->sendSpeedData (left_speed, right_speed, max_power, min_speed);
    ROS_INFO("speed send: %lf %lf", left_speed, right_speed);
#endif
}

void Hoverboard::tick() {
    api->protocolTick();
}
