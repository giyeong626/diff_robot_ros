#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "src/robotcontroller.cc"

RobotController robotcontroller;

void chatterCallback(const geometry_msgs::Twist::ConstPtr&msg){
    ROS_INFO("recieve msg = %f", msg-> linear.x);
    ROS_INFO("recieve msg = %f", msg-> linear.y);
    ROS_INFO("recieve msg = %f", msg-> linear.z);
    ROS_INFO("recieve msg = %f", msg-> angular.x);
    ROS_INFO("recieve msg = %f", msg-> angular.y);
    ROS_INFO("recieve msg = %f", msg-> angular.z);
    robotcontroller.Kinematics(msg-> linear.x, 0, msg-> angular.z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotcontroller");
    robotcontroller.ConnectUsb("/dev/ttyUSB0");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel",1000,chatterCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::Rate r(100.0);
    int count = 0;
    while (n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        
        robotcontroller.getOdometry(&vx, &vy, &vth);

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        last_time = current_time;
        count ++;
        r.sleep();
    }
    ros::spin();

    return 0;


}