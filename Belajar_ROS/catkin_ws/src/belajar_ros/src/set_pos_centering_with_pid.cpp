/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include "../include/belajar_ros/pid.hpp"

const double EULER = 2.71828182845904523536;

struct mission {
    double x;
    double y;
    bool done;
    int HMin;
    int SMin;
    int VMin;
    int HMax;
    int SMax;
    int VMax;
    bool scanning;
};

struct mission missions[4] = {
    {-4.3, 0, false, 127, 180, 0, 144, 255, 255, false}, // Pilar Biru => x, y, done, hmin, smin, vmin, hmax, smax, vmax, scanning
    {4.65, -11.2, false, 45, 110, 0, 50, 255, 255, false}, // Pilar Hijau
    {16.2, .1, false, 177, 190, 0, 179, 255, 255, false}, // Pilar Merah
    {5, 11.6, false, 140, 190, 0, 165, 255, 255, false} // Pilar Ungu
};

PID pid_x(0.03125, 1, -1, 0.5, 0.001, 0.001);
PID pid_y(0.03125, 1, -1, 0.5, 0.001, 0.001);
PID pid_z(0.03125, 1, -1, 0.5, 0.001, 0.001); 

PID pid_center_x(0.03125, 1, -1, 0.65, 0.001, 0.05);
PID pid_center_y(0.03125, 1, -1, 0.65, 0.001, 0.05);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::Pose center_pos;
void center_pub_cb(const geometry_msgs::Pose::ConstPtr& msg){
    center_pos = *msg;
}

geometry_msgs::Pose distance_to_center;
void distance_pub_cb(const geometry_msgs::Pose::ConstPtr& msg){
    distance_to_center = *msg;
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

    // Looping hingga took off
    while(ros::ok() && !tookOff)
    {
        ros::spinOnce();
        rate.sleep();

        // Jika belum mendekati z => 6 maka gerak ke z => 6
        if(!near_equal(current_pose.pose.position.z, 6, 0.3))
        {
            vel.linear.x = pid_x.calculate(current_pose.pose.position.x, current_pose.pose.position.x);
            vel.linear.y = pid_y.calculate(current_pose.pose.position.y, current_pose.pose.position.y);
            vel.linear.z = pid_z.calculate(6, current_pose.pose.position.z);

            cmd_pub.publish(vel);
            continue;
        }

        // Mulai perhitungan waktu
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

        // Jika sudah melewati 5 detik maka sudah bisa dianggap stabil dan sudah take off
        tookOff = true;
    }
}

void goLand(ros::Publisher cmd_pub, ros::Rate rate)
{
    ros::Time landing;
    geometry_msgs::Twist vel;
    bool landed = false;

    // Looping hingga landed
    while(ros::ok() && !landed)
    {
        ros::spinOnce();
        rate.sleep();

        // Jika belum mendekati posisi x,y => 0
        if(!near_equal(current_pose.pose.position.x, 0, 0.3) || !near_equal(current_pose.pose.position.y, 0, 0.3))
        {
            vel.linear.x = pid_x.calculate(0, current_pose.pose.position.x);
            vel.linear.y = pid_y.calculate(0, current_pose.pose.position.y);
            vel.linear.z = pid_z.calculate(current_pose.pose.position.z, current_pose.pose.position.z);

            // Mulai pindahin mendekati posisi x,y => 0
            cmd_pub.publish(vel);
            continue;
        }

        // Mengecek apakah sudah mulai hitung waktu
        if(landing.is_zero())
        {
            landing = ros::Time::now();
            continue;
        }
            
        cmd_pub.publish(vel);

        // Jika sudah melewati 5 detik maka mulai untuk landing ke z => 0
        if(ros::Time::now() - landing < ros::Duration(5.0))
        {
            continue;
        }

        // Bergerak ke z => 0 jika belum mendekati
        if(!near_equal(current_pose.pose.position.z, 0, 0.3))
        {
            vel.linear.x = pid_x.calculate(current_pose.pose.position.x, current_pose.pose.position.x);
            vel.linear.y = pid_y.calculate(current_pose.pose.position.y, current_pose.pose.position.y);
            vel.linear.z = pid_z.calculate(0, current_pose.pose.position.z);

            cmd_pub.publish(vel);
            // ROS_INFO("Going Home! | Going to z:%f", pose.pose.position.z);
            continue;
        } else {
            // Jika sudah mendekati titik z =>0 maka sudah bisa dianggap landed
            landed = true;
        }
    }
}

void goMission(ros::Publisher cmd_pub, ros::Rate rate, ros::Publisher hsv_pub, ros::Publisher scan_pub)
{
    ros::Time empty;
    ros::Time misi;
    geometry_msgs::Twist vel;
    int missionsDone = 0;
    int totalMissions = (sizeof(missions) / sizeof(missions[0]));
    bool centering = false;
    bool centered = false;

    // Looping hingga semua misi sukses
    while(ros::ok() && missionsDone != totalMissions)
    {
        ros::spinOnce();
        rate.sleep();

        // Looping setiap misi, dimulai dari misi setelah misi terakhir yang sukses
        for(int x = missionsDone; x < totalMissions; x++)
        {

            // Jika misi yang dicek sekarang belum selesai
            if(!missions[x].done)
            {
                vel.linear.x = 0;
                vel.linear.y = 0;
                vel.linear.z = 0;

                // Jika belum mode centering, maka gerak ke rough estimation center of pilar
                if(!centering)
                {
                    vel.linear.x = pid_x.calculate(missions[x].x, current_pose.pose.position.x);
                    vel.linear.y = pid_y.calculate(missions[x].y, current_pose.pose.position.y);
                    vel.linear.z = pid_z.calculate(6, current_pose.pose.position.z);
                }

                // Jika berada 2 satuan dari titik estimasi, maka mulai mode centering
                if(!centering && near_equal(current_pose.pose.position.x, missions[x].x, 2) && near_equal(current_pose.pose.position.y, missions[x].y, 2))
                {
                    centering = true;

                    // Mulai scanning, dengan mem-publish nilai hsv
                    if(!missions[x].scanning)
                    {
                        std_msgs::Bool scan;
                        scan.data = true;
                        missions[x].scanning = true;
                        scan_pub.publish(scan);
                        std_msgs::Int32MultiArray hsv_value;
                        hsv_value.data.resize(6);
                        hsv_value.data[0] = missions[x].HMin;
                        hsv_value.data[1] = missions[x].SMin;
                        hsv_value.data[2] = missions[x].VMin;
                        hsv_value.data[3] = missions[x].HMax;
                        hsv_value.data[4] = missions[x].SMax;
                        hsv_value.data[5] = missions[x].VMax;
                        hsv_pub.publish(hsv_value);
                        ROS_INFO("Published HSV!");
                        continue;
                    }
                }

                // Jika suatu dan beberapa hal, drone keluar 3 satuan dari radius titik estimasi, maka mulai balik lagi ke titik estimasi
                if(!near_equal(current_pose.pose.position.x, missions[x].x, 3) && !near_equal(current_pose.pose.position.y, missions[x].y, 3))
                {
                    centering = false;
                }

                // Jika dalam mode centering dan belum mendekati "centered"
                if(centering && !centered)
                {
                    // Gerakin drone dengan smooth, Konstan di sini arbitrary, makin besar makin pelan
                    double center_x = (center_pos.position.y * fabs(distance_to_center.position.y) / 120);
                    double center_y = (center_pos.position.x * fabs(distance_to_center.position.x) / 120);
                    vel.linear.x = pid_center_x.calculate(current_pose.pose.position.x - center_x, current_pose.pose.position.x);
                    vel.linear.y = pid_center_y.calculate(current_pose.pose.position.y - center_y, current_pose.pose.position.y);
                    vel.linear.z = pid_z.calculate(6, current_pose.pose.position.z);

                    // Jika jarak dari titika pusat sekitar 15 satuan, maka bisa dikatakan mendekati "centered"
                    if(distance_to_center.position.z <= 15)
                    {
                        centered = true;
                        continue;
                    }
                    ROS_INFO("Centering x: %f y:%f vel x:%f, y:%f", current_pose.pose.position.x - center_pos.position.x, current_pose.pose.position.y - center_pos.position.y, center_x, center_y);
                } else if(centered) {

                    // Sudah mendekati "centered" jadi dibutuhkan gerakan yang lebih cepat karena jarak yang sudah sangat kecil, konstan di sini arbitrary
                    double center_x = (center_pos.position.y * fabs(distance_to_center.position.y) / 60);
                    double center_y = (center_pos.position.x * fabs(distance_to_center.position.x) / 60);
                    vel.linear.x = pid_center_x.calculate(current_pose.pose.position.x - center_x, current_pose.pose.position.x);
                    vel.linear.y = pid_center_y.calculate(current_pose.pose.position.y - center_y, current_pose.pose.position.y);
                    vel.linear.z = pid_z.calculate(6, current_pose.pose.position.z);

                    // Mulai perhitungan timer
                    if(misi.is_zero())
                    {
                        misi = ros::Time::now();
                        ROS_INFO("Sampai di tujuan");
                    }

                    // Jika sudah lima detik
                    if(ros::Time::now() - misi > ros::Duration(5))
                    {
                        // Lanjutkan misi selanjutnya
                        ROS_INFO("Sudah 5 detik");
                        missions[x].done = true;
                        missionsDone++;
                        misi = empty;
                        

                        std_msgs::Bool scan;
                        scan.data = false;
                        scan_pub.publish(scan);

                        centered = false;
                        centering = false;
                        ROS_INFO("WP done : %d / %d", missionsDone, totalMissions);
                    }
                }
                break;
            }
        }
        
        cmd_pub.publish(vel);
    }
}

int main(int argc, char **argv)
{
    // Inisiasi node
    ros::init(argc, argv, "set_pos_centering_with_pid");
    // Inisiasi Node Handler
    ros::NodeHandle nh;

    // Kumpulan Subscriber dan Publisher
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
    
    // Publisher untuk mengirimkan Hue, Saturation, Value yang akan discan
    ros::Publisher hsv_pub = nh.advertise<std_msgs::Int32MultiArray>
            ("hsv_pub", 10);

    // Publisher untuk memilai scan
    ros::Publisher scan_pub = nh.advertise<std_msgs::Bool>
            ("scan_pub", 10);

    // Subscriber yang berisi nilai normalized vector
    ros::Subscriber center_pub = nh.subscribe<geometry_msgs::Pose>
            ("center_pub", 10, center_pub_cb);

    // Subscriber yang memberikan angka distance ke titik pusat x dan y, sedangkan z merupakan jarak antara koordinat pusat blob degan koordinat pusat kamera
    ros::Subscriber distance_pub = nh.subscribe<geometry_msgs::Pose>
            ("distance_pub", 10, distance_pub_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    ROS_INFO("Waiting for FCU Connection");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Mengirimkan beberapa setpoint awal untuk keep-up dengan rate
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = current_pose.pose.position.x;
    pose.pose.position.y = current_pose.pose.position.y;
    pose.pose.position.z = current_pose.pose.position.z;

    //send a few setpoints before starting
    ROS_INFO("Sending few setpoints before start");
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set mode ke offboard nantinya
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Set arming nantinya
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Menyimpan kapan terakhir request, digunakan saat arming dan offboard saja
    ros::Time last_request = ros::Time::now();

    // Menyimpan status arming
    bool armed = false;
    ROS_INFO("Starting");

    // Looping hingga armed
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

    // Mulai take off
    ROS_INFO("Taking Off! | Break a leg!");
    goTakeOff(cmd_pub, rate);

    // Mulai jalankan misi
    ROS_INFO("Starting Mission! | Finger crossed!");
    goMission(cmd_pub, rate, hsv_pub, scan_pub);

    // Mulai kembali ke titik awal
    ROS_INFO("Going Home! | Yay");
    goLand(cmd_pub, rate);

    ROS_INFO("Mission Done! | Woohoo!");
    return 0;
}
