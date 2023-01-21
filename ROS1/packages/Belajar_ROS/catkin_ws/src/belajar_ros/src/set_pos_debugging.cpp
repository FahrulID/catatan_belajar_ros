/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32MultiArray.h>
#include <math.h>
#include "../include/belajar_ros/pid.hpp"

PID pid_x(0.03125, 1, -1, 0.5, 0.001, 0.001);
PID pid_y(0.03125, 1, -1, 0.5, 0.001, 0.001);
PID pid_z(0.03125, 1, -1, 0.5, 0.001, 0.001); 

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

bool near_equal(double a, double b, double precision)
{
    return std::fabs(a - b) < precision;
}

void goTakeOff(ros::Publisher cmd_pub, ros::Rate rate)
{
    geometry_msgs::Twist vel;
    bool tookOff = false;
    ros::Time takingoff;

    while(ros::ok() && !tookOff)
    {
        ros::spinOnce();
        rate.sleep();

        if(!near_equal(current_pose.pose.position.z, 6, 0.2))
        {
            vel.linear.x = pid_x.calculate(current_pose.pose.position.x, current_pose.pose.position.x);
            vel.linear.y = pid_y.calculate(current_pose.pose.position.y, current_pose.pose.position.y);
            vel.linear.z = pid_z.calculate(6, current_pose.pose.position.z);

            cmd_pub.publish(vel);
            continue;
        }

        if(takingoff.is_zero())
        {
            takingoff = ros::Time::now();
            continue;
        }

        cmd_pub.publish(vel);

        if(ros::Time::now() - takingoff < ros::Duration(5.0))
        {
            continue;
        }

        tookOff = true;
    }
}

void goMission(ros::Publisher cmd_pub, ros::Rate rate, float position[3])
{
    geometry_msgs::Twist vel;

    while(ros::ok())
    {
        vel.linear.x = pid_x.calculate(position[0], current_pose.pose.position.x);
        vel.linear.y = pid_y.calculate(position[1], current_pose.pose.position.y);
        vel.linear.z = pid_z.calculate(position[2], current_pose.pose.position.z);
        cmd_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "set_pos_debugging");
    ros::NodeHandle param("~");
    float posX;
    float posY;
    float posZ;
    param.getParam("x", posX);
    param.getParam("y", posY);
    param.getParam("z", posZ);

    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Waiting for FCU Connection");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
    ROS_INFO("Sending few setpoints before start");
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool armed = false;
    ROS_INFO("Starting");
    while(ros::ok() && !armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    armed = true;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Taking Off! | Break a leg!");
    goTakeOff(cmd_pub, rate);

    float position[3] = {posX, posY, posZ};
    ROS_INFO("Starting Debugging! | Happy Debugging! x:%f y:%f z:%f", posX, posY, posZ);
    goMission(cmd_pub, rate, position);
    return 0;
}
