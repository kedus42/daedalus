#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "cstdlib"

#define MAX_SWING_DIST 1.0
#define CTRL_UPDATE_RATE 10.0

class holdCourse{
    protected:
        ros::NodeHandle nh;
        ros::Publisher drive_command_pub;
        ros::Publisher pid_pub;
        ros::Timer controller_timer;
        ros::Subscriber target_sub;
        geometry_msgs::Vector3 latest_orientation;
        std_msgs::Float64 latest_target;
        float integrated_error;
        float previous_error;
        float error_derivative;
        float kp;
        float ki;
        float kd;
        boost::shared_ptr<const geometry_msgs::Vector3> latest_shared_orientation;
        bool controller_on;
    public:
        holdCourse(){
            drive_command_pub = nh.advertise<std_msgs::Float64>("/daedalus_battery_steer_controller/command", 1, false);
            pid_pub = nh.advertise<geometry_msgs::Vector3>("/daedalus/pid/yaw", 1, false);
            target_sub = nh.subscribe("/daedalus/target", 1, &holdCourse::target_callback, this);
            controller_timer=nh.createTimer(ros::Duration(1.0/CTRL_UPDATE_RATE), &holdCourse::controller_callback, this);
            latest_shared_orientation = ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
            if (latest_shared_orientation!=NULL){
                latest_orientation=*latest_shared_orientation;
                latest_target.data=latest_orientation.z;
            }
            controller_on = false;
            integrated_error=0;
            error_derivative=0;
            previous_error=0;
            //Ziegler–Nichols Classic pid
            // kp=0.3;
            // ki=0.17;
            // kd=1.5;
            kp=1;
            ki=0.01;
            kd=0.0;
        }
        void target_callback(const std_msgs::Float64ConstPtr& new_target){
            set_target(*new_target);
            integrated_error=0;
        }
        void controller_callback(const ros::TimerEvent&){
            if (controller_on){
                latest_shared_orientation = ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
                if (latest_shared_orientation!=NULL){
                    latest_orientation=*latest_shared_orientation;
                }
                float error;
                error = latest_target.data - latest_orientation.z;
                if (error>180)
                    error-=360;
                else if (error<-180)
                    error+=360;
                geometry_msgs::Vector3 controller_output;
                controller_output.x=error*kp;
                integrated_error+=error*(1.0/CTRL_UPDATE_RATE);
                controller_output.y=integrated_error*ki;
                error_derivative=(error-previous_error)*CTRL_UPDATE_RATE;
                controller_output.z=error_derivative*kd;
                pid_pub.publish(controller_output);
                previous_error=error;
                error=kp*error+ki*integrated_error+kd*error_derivative;
                if (error > 180)
                    error = 180;
                else if(error < -180)
                    error = -180;
                std_msgs::Float64 steer_correction;
                steer_correction.data=(error/180)*MAX_SWING_DIST;
                drive_command_pub.publish(steer_correction);
            }
        }
        void set_target(const std_msgs::Float64 new_relative_target){
            latest_shared_orientation = ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
            if (latest_shared_orientation!=NULL){
                latest_orientation=*latest_shared_orientation;
            }
            latest_target.data=latest_orientation.z+new_relative_target.data;
            if (latest_target.data>180)
                latest_target.data=latest_target.data-360;
            else if(latest_target.data<-180)
                latest_target.data=360+latest_target.data;
        }
        void set_controller(bool state){
            controller_on=state;
        }
        void reset_controller(){
            integrated_error=0;
            previous_error=0;
            error_derivative=0;
        }
    };

int main(int argc, char** argv){
    ros::init(argc, argv, "course_controller");
    holdCourse controller_object;
    bool controller_on;
    std_msgs::Float64 new_target;
    while(ros::ok()){
        ros::param::get("/controller_on", controller_on);
        if (controller_on){
            controller_object.set_controller(true);
        } else{
            controller_object.set_controller(false);
            controller_object.reset_controller();
        }
        ros::spinOnce();
    }
    return 0;
}