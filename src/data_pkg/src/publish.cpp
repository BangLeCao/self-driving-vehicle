#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "../include/serial_helper.h"

void dataCallback(const std_msgs::Float64MultiArray &msg) 
{
    std::vector<double> my_data = msg.data;
    for (std::vector<double>::iterator it = my_data.begin();
        it != my_data.end(); ++it)
    {
        ROS_INFO("%lf", *it);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("VDATA", 10);
    ros::Subscriber sub = n.subscribe("PCDAT", 10, dataCallback);
    ros::Rate loop_rate(10); //10Hz
    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;
        std::vector<double> pub_data{1, 2.2, 3.3, 4};
        for (std::vector<double>::iterator it = pub_data.begin();
        it != pub_data.end(); ++it)
        {
            msg.data.push_back(*it);
        }
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}