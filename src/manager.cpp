#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <string>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_manager");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    std::string serial_name = nh.param<std::string>("serial_name", "/dev/ttyUSB0");
    int baud_rate = nh.param<int>("baud_rate", 115200);

    serial::Serial serial_port;
    serial::Timeout timeout;
    timeout = serial::Timeout::simpleTimeout(100);

    ros::Publisher serial_msg_pub = nh.advertise<std_msgs::String>("/serial_msg_RX", 1);
    ros::Subscriber serial_msg_sub = nh.subscribe<std_msgs::String>("/serial_msg_TX", 1, [&](const std_msgs::String::ConstPtr &msg) {
        serial_port.write(msg->data);
    });

    try {
        serial_port.open(); // 尝试打开串口
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open the serial.Please check your settings.");
        return -1;
    }
    ROS_INFO("Chassis serial has been open.");

    while (ros::ok())
    {
        if (serial_port.available())
        {
            std_msgs::String msg;
            msg.data = serial_port.read(serial_port.available());
            serial_msg_pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}