#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"

#define TICKS_PER_MTR 6.0
#define POSE_UPDATE_RATE 10.0
#define IMU_UPDATE_RATE 100.0
#define FILTER_ALPHA 0.1
#define COOLDOWN_DURATION 5.0

class encoderToPose{
    protected:
        geometry_msgs::PoseStamped pose_estimate;
        geometry_msgs::Vector3 latest_euler_orientation;
        geometry_msgs::Vector3 zpos_debug;
        sensor_msgs::Imu latest_imu_reading;
        tf::Quaternion latest_quat_orientaion;
        ros::NodeHandle nh;
        ros::Subscriber orientation_sub;
        ros::Subscriber encoder_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber drive_sub;
        ros::Publisher pose_pub;
        ros::Publisher debug_pub;
        ros::Timer update_timer;
        ros::Timer cooldown_timer;
        int prev_pos;
        int current_pos;
        float zaccel;
        float zaccel_prev;
        float zvel;
        float zpos;
        bool toggle_integration;
        bool toggle_cooldown;
    public:
        encoderToPose(){
            orientation_sub=nh.subscribe<geometry_msgs::Vector3>("/daedalus/euler_orientation", 1, &encoderToPose::update_orientation, this);
            encoder_sub = nh.subscribe<std_msgs::Int64>("/daedalus/right_drive_pos", 1, &encoderToPose::encoder_callback, this);
            //encoder_sub = nh.subscribe<std_msgs::Int64>("/daedalus/left_drive_pos", 1, &encoderToPose::encoder_callback, this);
            imu_sub = nh.subscribe<sensor_msgs::Imu>("/daedalus/imu_0", 1, &encoderToPose::imu_callback, this);
            drive_sub = nh.subscribe<std_msgs::Float64MultiArray>("/daedalus_battery_velocity_controller/command", 1, &encoderToPose::drive_callback, this);
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/daedalus/pose_estimate", 1, false);
            debug_pub = nh.advertise<geometry_msgs::Vector3>("/daedalus/zpos_debug", 1, false);
            cooldown_timer = nh.createTimer(ros::Duration(COOLDOWN_DURATION), &encoderToPose::cooldown_callback, this, true, false);
            update_timer = nh.createTimer(ros::Duration(1.0/POSE_UPDATE_RATE), &encoderToPose::update_pose, this);
            prev_pos = 0;
            current_pos = 0;
            zvel = 0;
            zpos = 0;
            zaccel = 0;
            zaccel_prev = 0;
            toggle_integration = false;
            toggle_cooldown = false;
        }
        void update_orientation(const geometry_msgs::Vector3ConstPtr & latest){
            latest_euler_orientation=*latest;
            latest_quat_orientaion.setRPY((*latest).x, (*latest).y, (*latest).z);
        }
        void encoder_callback(const std_msgs::Int64ConstPtr & latest){
            latest_quat_orientaion.normalize();
            tf::quaternionTFToMsg(latest_quat_orientaion, pose_estimate.pose.orientation);
            current_pos = (*latest).data;
            if (current_pos == 0 && prev_pos == 7){
                pose_estimate.pose.position.x+=1.0/TICKS_PER_MTR*cos(latest_euler_orientation.z*3.14/180);
                pose_estimate.pose.position.y+=1.0/TICKS_PER_MTR*sin(latest_euler_orientation.z*3.14/180);
            } else{
                pose_estimate.pose.position.x+=(current_pos - prev_pos)*1.0/TICKS_PER_MTR*cos(latest_euler_orientation.z*3.14/180);
                pose_estimate.pose.position.y+=(current_pos - prev_pos)*1.0/TICKS_PER_MTR*sin(latest_euler_orientation.z*3.14/180);
            }
            pose_estimate.header.stamp.sec=ros::Time::now().sec;
            pose_estimate.header.stamp.nsec=ros::Time::now().nsec;
            prev_pos=current_pos;
        }
        void cooldown_callback(const ros::TimerEvent &){
            toggle_cooldown = true;
        }
        void drive_callback(const std_msgs::Float64MultiArrayConstPtr & command){
            if ((*command).data[0] != 0){
                toggle_integration = true;
                cooldown_timer.setPeriod(ros::Duration(COOLDOWN_DURATION));
                cooldown_timer.start();
            }
            else{
                toggle_integration = false;
                toggle_cooldown = false;
            }
        }
        void imu_callback(const sensor_msgs::ImuConstPtr & latest){
            latest_imu_reading = *latest;
            zaccel = FILTER_ALPHA*((*latest).linear_acceleration.z-9.8) + (1-FILTER_ALPHA)*zaccel_prev;
            if (toggle_integration && toggle_cooldown)
                zvel += 0.5*(zaccel_prev+zaccel)*(1.0/IMU_UPDATE_RATE);
            else
                zvel = 0;
            pose_estimate.pose.position.z += zvel*(1.0/IMU_UPDATE_RATE);
            zaccel_prev = zaccel;
            zpos_debug.x=zaccel;
            zpos_debug.y=zvel;
            zpos_debug.z=pose_estimate.pose.position.z;
            debug_pub.publish(zpos_debug);
        }
        void update_pose(const ros::TimerEvent &){
            pose_pub.publish(pose_estimate);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_based_position_estimator");
    encoderToPose EtP;
    ros::spin(); 
    return 0;
} 