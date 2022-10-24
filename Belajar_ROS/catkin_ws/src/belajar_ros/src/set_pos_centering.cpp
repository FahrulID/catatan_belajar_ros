/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

struct mission {
    double x;
    double y;
    bool done;
};

struct mission missions[4] = {
    {-4.3, 0, false}, // Pilar Biru
    {4.65, -11.2, false}, // Pilar Hijau
    {16.2, .1, false}, // Pilar Merah
    {5, 11.6, false} // Pilar Ungu
};

bool near_equal(double a, double b, double precision)
{
    return std::fabs(a - b) < precision;
}

void goTakeOff(ros::Publisher local_pos_pub, ros::Rate rate)
{
    geometry_msgs::PoseStamped pose;
    bool tookOff = false;
    ros::Time takingoff;

    while(ros::ok() && !tookOff)
    {
        ros::spinOnce();
        rate.sleep();

        if(!near_equal(current_pose.pose.position.z, 5, 0.1))
        {
            pose.pose.position.x = current_pose.pose.position.x;
            pose.pose.position.y = current_pose.pose.position.y;
            pose.pose.position.z = 5;

            local_pos_pub.publish(pose);
        }

        if(takingoff.is_zero())
        {
            takingoff = ros::Time::now();
            continue;
        }
            
        local_pos_pub.publish(pose);

        if(ros::Time::now() - takingoff < ros::Duration(5.0))
        {
            continue;
        }

        tookOff = true;
    }
}

void goLand(ros::Publisher local_pos_pub, ros::Rate rate)
{
    ros::Time landing;
    geometry_msgs::PoseStamped pose;
    bool landed = false;

    while(ros::ok() && !landed)
    {
        ros::spinOnce();
        rate.sleep();
        if(!near_equal(current_pose.pose.position.x, 0, 0.1) || !near_equal(current_pose.pose.position.y, 0, 0.1))
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = current_pose.pose.position.z;
            
            local_pos_pub.publish(pose);
            // ROS_INFO("Going Home! | Going to x:%f y:%f", pose.pose.position.x, pose.pose.position.y);
            continue;
        }

        if(landing.is_zero())
        {
            landing = ros::Time::now();
            continue;
        }
            
        local_pos_pub.publish(pose);

        if(ros::Time::now() - landing < ros::Duration(5.0))
        {
            continue;
        }

        if(!near_equal(current_pose.pose.position.z, 0, 0.1))
        {
            pose.pose.position.x = current_pose.pose.position.x;
            pose.pose.position.y = current_pose.pose.position.y;
            pose.pose.position.z = 0;

            local_pos_pub.publish(pose);
            // ROS_INFO("Going Home! | Going to z:%f", pose.pose.position.z);
            continue;
        } else {
            landed = true;
        }
    }
}

void goMission(ros::Publisher local_pos_pub, ros::Rate rate)
{
    ros::Time empty;
    ros::Time misi;
    geometry_msgs::PoseStamped pose;
    int missionsDone = 0;
    int totalMissions = (sizeof(missions) / sizeof(missions[0]));

    while(ros::ok() && missionsDone != totalMissions)
    {
        ros::spinOnce();
        rate.sleep();

        for(int x = missionsDone; x < totalMissions; x++)
        {
            if(!missions[x].done)
            {
                pose.pose.position.x = missions[x].x;
                pose.pose.position.y = missions[x].y;
                pose.pose.position.z = 5;
                if(near_equal(current_pose.pose.position.x, missions[x].x, 0.2) && near_equal(current_pose.pose.position.y, missions[x].y, 0.2))
                {
                    if(misi.is_zero())
                    {
                        misi = ros::Time::now();
                        ROS_INFO("Sampai di tujuan");
                    }

                    if(ros::Time::now() - misi > ros::Duration(5))
                    {
                        missions[x].done = true;
                        missionsDone++;
                        misi = empty;
                        ROS_INFO("WP done : %d / %d", missionsDone, totalMissions);
                    }
                }
                break;
            }
        }
        
        local_pos_pub.publish(pose);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_pos_centering");
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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting
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
    goTakeOff(local_pos_pub, rate);

    ROS_INFO("Starting Mission! | Finger crossed!");
    goMission(local_pos_pub, rate);

    ROS_INFO("Going Home! | Yay");
    goLand(local_pos_pub, rate);

    ROS_INFO("Mission Done! | Woohoo!");
    return 0;
}
