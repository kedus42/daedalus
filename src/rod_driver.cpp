//class member array storing how long each rod should be
//timer triggers the looping of this array to publishing extension msgs based on array
//cli commands now , vpip triggered by timer later sets the values in this array
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "daedalus/Extension.h"
#include "daedalus/VPIP.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include "math.h"

class Driver{
    private:
        ros::NodeHandle nh_;
        std_msgs::Float32MultiArray extensions_;
        daedalus::Extension extension_cmd_;
        ros::Subscriber cmd_sub_;
        ros::Subscriber vpip_sub_;
        ros::Subscriber euler_sub_;
        ros::Subscriber joint_sub_;
        ros::Publisher ext_pub_;
        ros::Timer execution_timer_;
        ros::Timer vpip_timer_;
        float drive_pos;
        float roll;
        float pitch;
        float sphere_roll_;
        float sphere_pitch_;
        float rs_;
        float rm_;
        float lm_;
        float drive_pos_;
        void cmd_callback_(const std_msgs::StringConstPtr &cmd){
            int i =0;
            if (cmd->data == "forward"){
                while (i<16){
                    extensions_.data[i] = 0;
                    i++;
                }
                extensions_.data[5] = .28;
                extensions_.data[4] = .28;
                extensions_.data[13] = .28;
                extensions_.data[12] = .28;
            } else if(cmd->data == "backward"){
                while (i<16){
                    extensions_.data[i] = 0;
                    i++;
                }
                extensions_.data[3] = .28;
                extensions_.data[4] = .28;
                extensions_.data[11] = .28;
                extensions_.data[12] = .28;
            } else if(cmd->data == "right"){
                while (i<16){
                    extensions_.data[i] = 0;
                    i++;
                }
                extensions_.data[5] = .28;
                extensions_.data[4] = .28;
            } else if(cmd->data == "left"){
                while (i<16){
                    extensions_.data[i] = 0;
                    i++;
                }
                extensions_.data[13] = .28;
                extensions_.data[12] = .28;
            }
        }
        void vpip_callback_(const daedalus::VPIPConstPtr &vpip){
            roll = vpip->roll*M_PI /180;
            pitch = vpip->pitch*M_PI /180;
            
        }
        void euler_callback_(const geometry_msgs::Vector3ConstPtr &rpy){
            sphere_roll_ = rpy->x;
            sphere_roll_ = sphere_roll_ *M_PI /180;
            sphere_pitch_ = rpy->y;
            sphere_pitch_ = sphere_pitch_ *M_PI /180;
        }
        void timer_callback_(const ros::TimerEvent&){
            int i =0;
            while (i<8){
                extension_cmd_.side = 0;
                extension_cmd_.index = i;
                extension_cmd_.ext = extensions_.data[i];
                ext_pub_.publish(extension_cmd_);
                //sleep(0.15);
                i++;
            }
            while (i<16){
                extension_cmd_.side = 1;
                extension_cmd_.index = i-8;
                extension_cmd_.ext = extensions_.data[i];
                ext_pub_.publish(extension_cmd_);
                //sleep(0.15);
                i++;
            }
        }
        void joint_callback_(const sensor_msgs::JointStateConstPtr &state){
            drive_pos = state->position[53];
        }
        void vpip_timer_callback_(const ros::TimerEvent&){
            int i =0;
            int sf = 1;
            if(roll == 0 && pitch == 0)
                return;
            while(i<8){
                float theta = -1*drive_pos+0.2+((8-i)*M_PI/4);
                //float theta = -drive_pos + 0.2 + (8-i)*M_PI/4;

                float dms = sin(acos(rs_/rm_)) * rm_;
                float dmtx = - cos(theta) * sin(sphere_roll_);
                float dmty = - sin(theta);
                float dmtz = - cos(theta)*cos(sphere_roll_);  

                float first_term = -tan(roll)*sf*cos(sphere_roll_)*dms-rm_+sf*sin(sphere_roll_)*dms;
                float second_term = tan(roll)*dmtx+tan(pitch)*dmty+dmtz;

                extensions_.data[i] =  ((-tan(roll)*sf*cos(sphere_roll_)*dms-rm_+sf*sin(sphere_roll_)*dms)/(tan(roll)*dmtx+tan(pitch)*dmty+dmtz))-rs_;
                //extensions_.data[i] = extensions_.data[i] * -1 + rm_;

                ROS_INFO("flat ground: %f ", rm_/cos(theta)-rs_);
                ROS_INFO("extesnion: %f", extensions_.data[i]);
                ROS_INFO("theta: %f", fmod(theta*180/M_PI,360));
                if (extensions_.data[i]>lm_ || extensions_.data[i] < 0)
                    extensions_.data[i] = 0;
                i++;
            }
            while(i<16){
                sf = -1;
                float theta = -1*drive_pos+0.2+((16-i)*M_PI/4);
                //float theta = -drive_pos + 0.2 + (16-i)*M_PI/4;

                float dms = sin(acos(rs_/rm_)) * rm_;
                float dmtx = - cos(theta) * sin(sphere_roll_);
                float dmty = - sin(theta);
                float dmtz = - cos(theta)*cos(sphere_roll_);  

                float first_term = -tan(roll)*sf*cos(sphere_roll_)*dms-rm_+sf*sin(sphere_roll_)*dms;
                float second_term = tan(roll)*dmtx+tan(pitch)*dmty+dmtz;

                extensions_.data[i] = ((-tan(roll)*sf*cos(sphere_roll_)*dms-rm_+sf*sin(sphere_roll_)*dms)/(tan(roll)*dmtx+tan(pitch)*dmty+dmtz))-rs_;
                //extensions_.data[i] = extensions_.data[i] * -1 + rm_;

                ROS_INFO("flat ground: %f ", rm_/cos(theta)-rs_);
                ROS_INFO("extesnion: %f", extensions_.data[i]);
                ROS_INFO("theta: %f", fmod(theta*180/M_PI,360));
                if (extensions_.data[i]>lm_ || extensions_.data[i] < 0)
                    extensions_.data[i] = 0;
                i++;
            }
        }
    public:
        Driver(){
            cmd_sub_ = nh_.subscribe<std_msgs::String>("daedalus/rod_drive_cmd", 1, &Driver::cmd_callback_, this);
            vpip_sub_ = nh_.subscribe<daedalus::VPIP>("daedalus/vpip_description", 1, &Driver::vpip_callback_, this);
            euler_sub_ = nh_.subscribe<geometry_msgs::Vector3>("daedalus/euler_orientation", 3, &Driver::euler_callback_, this);
            joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 3, &Driver::joint_callback_, this);
            ext_pub_ = nh_.advertise<daedalus::Extension>("/daedalus/extend_rod", 16);
            execution_timer_ = nh_.createTimer(ros::Duration(0.5), &Driver::timer_callback_, this);
            vpip_timer_ = nh_.createTimer(ros::Duration(0.01), &Driver::vpip_timer_callback_, this);
            roll=0;
            pitch=0;
            rm_=0.232;
            rs_=0.07;
            lm_=0.28;
            int i =0;
            while (i<16){
                extensions_.data.push_back(0);
                i++;
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rod_driver");
    Driver driver;
    ros::spin(); 
    return 0;
}