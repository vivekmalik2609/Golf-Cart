#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char**argv){
ros::init(argc, argv, "ramp");
ros::NodeHandle nh;

ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
ros::Rate loop_rate(10);


while(ros::ok()){
int x=0;
int m=1;
geometry_msgs::Twist msg;
msg.linear.x = m*x;

pub.publish(msg);

ROS_INFO_STREAM("Sending ramp velocity command:"<<" linear="<<msg.linear.x<<);
x=x+0.05;

rate.sleep();
}
}
