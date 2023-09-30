#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8MultiArray.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_manager", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    std::string serial_name = nh.param<std::string>("serial_name", "/dev/ttyUSB0");
    std::string serial_rx_topic = nh.param<std::string>("RX_topic", "/serial_msg_RX");
    std::string serial_tx_topic = nh.param<std::string>("TX_topic", "/serial_msg_TX");
    int baud_rate = nh.param<int>("serial_baud", 115200);


    serial::Serial serial_port;
    serial::Timeout timeout;
    timeout = serial::Timeout::simpleTimeout(100);

    serial_port.setPort(serial_name);
    serial_port.setBaudrate(baud_rate);
    serial_port.setTimeout(timeout);

    try {
        serial_port.open(); // 尝试打开串口
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open the serial. Please check your settings.");
        return -1;
    }
    ROS_INFO("Chassis serial has been open.");

    ros::Publisher serial_msg_pub = nh.advertise<std_msgs::UInt8MultiArray>(serial_rx_topic, 1);
    ros::Subscriber serial_msg_sub = nh.subscribe<std_msgs::UInt8MultiArray>(serial_tx_topic, 1, 
        [&](const std_msgs::UInt8MultiArray::ConstPtr &msg) {
            serial_port.write(msg->data.data(), msg->data.size());
        });

    while (ros::ok())
    {
        if (serial_port.available())
        {
            std_msgs::UInt8MultiArray msg;
            while (serial_port.available())
            {
                msg.data.push_back(serial_port.read(1)[0]);
            }
            serial_msg_pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}