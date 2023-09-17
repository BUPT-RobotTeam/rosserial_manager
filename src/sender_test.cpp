#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sender_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher serial_msg_pub = nh.advertise<std_msgs::UInt8MultiArray>("/serial_msg_TX", 1);

    while (ros::ok())
    {
        std_msgs::UInt8MultiArray msg;
        msg.data.push_back(0x01);
        msg.data.push_back(0x02);
        msg.data.push_back(0x03);
        msg.data.push_back(0x04);
        msg.data.push_back(0x05);
        msg.data.push_back(0x06);
        serial_msg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}