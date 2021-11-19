#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#define ACCEPTABLE_IDLE_SPIN 15
#define UPDATE_RATE 2.0

class spinCorrector{
    protected:
        geometry_msgs::Vector3 latest_orientation;
        std_msgs::Float64MultiArray extend_legs;
        std_msgs::Float64MultiArray retract_legs;
        std_msgs::Bool toggle_pitch_switch;
        ros::NodeHandle nh;
        ros::Subscriber orientation_sub;
        ros::Subscriber switch_sub;
        ros::Publisher right_pub;
        ros::Publisher left_pub;
        ros::Publisher pitch_switch_pub;
        float setpoint;
        bool toggle_spin_control;
    public:
        spinCorrector(){
            toggle_pitch_switch.data=true;
            int iterator = 0;
            while (iterator<24){
                extend_legs.data.push_back(1);
                iterator+=1;
            }
            iterator = 0;
            while (iterator<24){
                retract_legs.data.push_back(0);
                iterator+=1;
            }
            orientation_sub = nh.subscribe<geometry_msgs::Vector3>("/daedalus/euler_orientation", 1, &spinCorrector::update_orientation, this);
            switch_sub = nh.subscribe<std_msgs::Bool>("/daedalus/switches/spin_control", 1, &spinCorrector::switch_callback, this);
            left_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_left_controller/command", 1, false);
            right_pub = nh.advertise<std_msgs::Float64MultiArray>("daedalus_right_controller/command", 1, false);
            pitch_switch_pub = nh.advertise<std_msgs::Bool>("/daedalus/switches/pitch_control", 1, false); 
        }
        void update_orientation(const geometry_msgs::Vector3ConstPtr & latest){
            latest_orientation = *latest;
        }
        void switch_callback(const std_msgs::BoolConstPtr & state){
            toggle_spin_control=(*state).data;
            if (toggle_spin_control){
                ROS_INFO("spin corrector on");
                loop_controller();
            }
            else{
                ROS_INFO("spin corrector off");
                left_pub.publish(retract_legs);
                right_pub.publish(retract_legs);
            }
        }
        void loop_controller(){
            setpoint = latest_orientation.z;
            ros::Rate rate(UPDATE_RATE);
            while(ros::ok() && toggle_spin_control){
                ROS_INFO("spin deviation: %f", setpoint - latest_orientation.z);
                if (setpoint-latest_orientation.z>ACCEPTABLE_IDLE_SPIN){
                    if (toggle_pitch_switch.data==true){
                        toggle_pitch_switch.data=false;
                        pitch_switch_pub.publish(toggle_pitch_switch);
                    }
                    ROS_INFO("deviation greater than positive acceptable idle spin");
                    right_pub.publish(extend_legs);
                }
                else if(setpoint-latest_orientation.z<-1*ACCEPTABLE_IDLE_SPIN){
                    if (toggle_pitch_switch.data==true){
                        toggle_pitch_switch.data=false;
                        pitch_switch_pub.publish(toggle_pitch_switch);
                    }
                    ROS_INFO("deviation less than negative acceptable idle spin");
                    left_pub.publish(extend_legs);
                } else{
                    if (toggle_pitch_switch.data==false){
                        toggle_pitch_switch.data=true;
                        pitch_switch_pub.publish(toggle_pitch_switch);
                    }
                }
                ros::spinOnce();
                rate.sleep();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spin_corrector");
    spinCorrector corrector;
    ros::spin(); 
    return 0;
} 