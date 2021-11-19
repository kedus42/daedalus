#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include "math.h"
#include "std_msgs/Int64.h"

#define CTRL_UPDATE_RATE 5.0
#define LEG_PLANTING_THRESHHOLD 45

class pitchCorrector{
    protected:
        std_msgs::Float64MultiArray extend_legs;
        std_msgs::Float64MultiArray left_leg_control;
        std_msgs::Float64MultiArray right_leg_control;
        std_msgs::Float64MultiArray retract_legs;
        boost::shared_ptr<const sensor_msgs::JointState> joint_states;
        boost::shared_ptr<const geometry_msgs::Vector3> latest_orientation;
        bool toggle_pitch_control;
        float left_drive_pos;
        float right_drive_pos;
        ros::NodeHandle nh;
        ros::Publisher left_even_pub;
        ros::Publisher right_even_pub;
        ros::Publisher left_uneven_pub;
        ros::Publisher right_uneven_pub;
        ros::Publisher left_pub;
        ros::Publisher right_pub;
        //ros::Subscriber pcl_sub;
        ros::Subscriber switch_sub;
        ros::Subscriber right_encoder_sub;
        ros::Subscriber left_encoder_sub;
    public:
        pitchCorrector(){
            //pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/daedalus/cloud/stitched", 1, &pitchCorrector::pcl_callback, this);
            switch_sub=nh.subscribe<std_msgs::Bool>("/daedalus/switches/pitch_control", 1, &pitchCorrector::activation_callback, this);
            right_encoder_sub=nh.subscribe<std_msgs::Int64>("/daedalus/right_drive_pos", 1, &pitchCorrector::right_pos_update, this);
            left_encoder_sub=nh.subscribe<std_msgs::Int64>("/daedalus/left_drive_pos", 1, &pitchCorrector::left_pos_update, this);
            left_even_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_leg_l_even_controller/command", 1, false);
            right_even_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_leg_r_even_controller/command", 1, false);
            left_uneven_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_leg_l_uneven_controller/command", 1, false);
            right_uneven_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_leg_r_uneven_controller/command", 1, false);
            left_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_left_controller/command", 1, false);
            right_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_right_controller/command", 1, false);
            int i=0;
            while(i<24){
                extend_legs.data.push_back(1);
                i++;
            }
            i=0;
            while(i<24){
                retract_legs.data.push_back(0);
                i++;
            }
            i=0;
            while(i<24){
                left_leg_control.data.push_back(0);
                i++;
            }
            i=0;
            while(i<24){
                right_leg_control.data.push_back(0);
                i++;
            }
            toggle_pitch_control=false;
        }
        //void pcl_callback(const sensor_msgs::PointCloud2ConstPtr & input_cloud){
            
        //}
        void activation_callback(const std_msgs::BoolConstPtr & state){
            toggle_pitch_control=(*state).data;
            if (toggle_pitch_control){
                ROS_INFO("pitch corrector on");
                loop_controller();
            }
            else{
                ROS_INFO("pitch corrector off");
                left_pub.publish(retract_legs);
                right_pub.publish(retract_legs);
            }
        }
        void left_pos_update(const std_msgs::Int64ConstPtr &latest){
            left_drive_pos=(*latest).data;
        }
        void right_pos_update(const std_msgs::Int64ConstPtr &latest){
            right_drive_pos=(*latest).data;
        }
        void loop_controller(){
            ros::Rate rate(CTRL_UPDATE_RATE);
            while(ros::ok() && toggle_pitch_control){
                latest_orientation=ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation");
                if ((*latest_orientation).y > LEG_PLANTING_THRESHHOLD || (*latest_orientation).y < -1*LEG_PLANTING_THRESHHOLD){
                    // joint_states=ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh);
                    // right_drive_pos=(*joint_states).position[53];
                    // left_drive_pos=(*joint_states).position[54];
                    // right_drive_pos=fmod(right_drive_pos, 6.28);
                    // left_drive_pos=fmod(left_drive_pos, 6.28);
                    // right_drive_pos=int(right_drive_pos*7/6.28);
                    // left_drive_pos=int(left_drive_pos*7/6.28);
                    int iterator = 0;
                    //int offset = 0;
                    //offset = (int((*latest_orientation).y) - LEG_PLANTING_THRESHHOLD)/10;
                    if((*latest_orientation).y < LEG_PLANTING_THRESHHOLD){
                        ROS_INFO("pitch too low");
                        while(iterator<3){
                            if (5+iterator-left_drive_pos>=0){
                                ROS_INFO("#1 left: actuating %f through %f", (5+iterator)*3-left_drive_pos*3, (5+iterator)*3-left_drive_pos*3+2);
                                left_leg_control.data[(5+iterator)*3-left_drive_pos*3]=1;
                                left_leg_control.data[(5+iterator)*3-left_drive_pos*3+1]=1;
                                left_leg_control.data[(5+iterator)*3-left_drive_pos*3+2]=1;
                            }
                            else {
                                ROS_INFO("#2 left: actuating %f through %f", 8*3-(5+iterator)*3-left_drive_pos*3, 8*3-(5+iterator)*3-left_drive_pos*3+2);
                                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)]=1;
                                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)+1]=1;
                                left_leg_control.data[8*3+((5+iterator)*3-left_drive_pos*3)+2]=1;  
                            }
                            if (5+iterator-right_drive_pos>=0){
                                ROS_INFO("#3 right: actuating %f through %f", (5+iterator)*3-right_drive_pos*3, (5+iterator)*3-right_drive_pos*3+2);
                                right_leg_control.data[(5+iterator)*3-right_drive_pos*3]=1;
                                right_leg_control.data[(5+iterator)*3-right_drive_pos*3+1]=1;
                                right_leg_control.data[(5+iterator)*3-right_drive_pos*3+2]=1;
                            }
                            else {
                                ROS_INFO("#4 right: actuating %f through %f", 8*3-(5+iterator)*3-right_drive_pos*3, 8*3-(5+iterator)*3-right_drive_pos*3+2);
                                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)]=1;
                                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)+1]=1;
                                right_leg_control.data[8*3+((5+iterator)*3-right_drive_pos*3)+2]=1;  
                            }
                            iterator++;
                        } 
                    } else{
                        ROS_INFO("pitch too high");
                        while(iterator<3){
                            if (1+iterator-left_drive_pos>=0){
                                ROS_INFO("#5 left: actuating %f through %f", (1+iterator)*3-left_drive_pos*3, (1+iterator)*3-left_drive_pos*3+1);
                                left_leg_control.data[(1+iterator)*3-left_drive_pos*3]=1;
                                left_leg_control.data[(1+iterator)*3-left_drive_pos*3+1]=1;
                                left_leg_control.data[(1+iterator)*3-left_drive_pos*3+2]=1;
                            }
                            else {
                                ROS_INFO("#6 left: actuating %f through %f", 8*3-(1+iterator)*3-left_drive_pos*3, 8*3-(1+iterator)*3-left_drive_pos*3+1);
                                left_leg_control.data[8*3+(1+iterator)*3-left_drive_pos*3]=1;
                                left_leg_control.data[8*3+(1+iterator)*3-left_drive_pos*3+1]=1;
                                left_leg_control.data[8*3+(1+iterator)*3-left_drive_pos*3+2]=1;  
                            }
                            if (1+iterator-right_drive_pos>=0){
                                ROS_INFO("#7 right: actuating %f through %f", (1+iterator)*3-right_drive_pos*3, (1+iterator)*3-right_drive_pos*3+1);
                                right_leg_control.data[(1+iterator)*3-right_drive_pos*3]=1;
                                right_leg_control.data[(1+iterator)*3-right_drive_pos*3+1]=1;
                                right_leg_control.data[(1+iterator)*3-right_drive_pos*3+2]=1;
                            }
                            else {
                                ROS_INFO("#8 right: actuating %f through %f", 8*3-(1+iterator)*3-right_drive_pos*3, 8*3-(1+iterator)*3-right_drive_pos*3+1);
                                right_leg_control.data[8*3+(1+iterator)*3-right_drive_pos*3]=1;
                                right_leg_control.data[8*3+(1+iterator)*3-right_drive_pos*3+1]=1;
                                right_leg_control.data[8*3+(1+iterator)*3-right_drive_pos*3+2]=1;  
                            }
                            iterator++;
                        } 
                    }
                    if(toggle_pitch_control){
                        left_pub.publish(left_leg_control);
                        right_pub.publish(right_leg_control);
                    }
                    iterator=0;
                    while (iterator < 24){
                        left_leg_control.data[iterator]=0;
                        right_leg_control.data[iterator]=0;
                        iterator++;
                    }
                } /*else{
                    left_pub.publish(retract_legs);
                    right_pub.publish(retract_legs);
                }*/
                ros::spinOnce();
                rate.sleep();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pitch_corrector");
    pitchCorrector corrector;
    ros::spin(); 
    return 0;
} 