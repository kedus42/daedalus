#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "daedalus/Extension.h"
#include "std_msgs/Int64.h"
#include "sensor_msgs/JointState.h"

class Allocator{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber extension_sub_;
        ros::Publisher left_pub_;
        ros::Publisher right_pub_;
        std_msgs::Float64MultiArray left_array_;
        std_msgs::Float64MultiArray right_array_;
        ros::Subscriber rod_pos_sub_;
        int left_drive_pos_;
        int right_drive_pos_;
        void extension_callback_(const daedalus::ExtensionConstPtr &extension_cmd){
            if(extension_cmd->side == 0 ){
                int i =0;
                int compensated_pos = extension_cmd->index- left_drive_pos_;
                if (compensated_pos < 0)
                    compensated_pos += 7;
                while(i<3){
                    left_array_.data[compensated_pos*3 + i] = extension_cmd->ext/3.0;
                    i++; 
                }
                left_pub_.publish(left_array_);
            } else{
                int i = 0;
                int compensated_pos = extension_cmd->index- right_drive_pos_;
                if (compensated_pos < 0)
                    compensated_pos += 7;
                while(i<3){
                    right_array_.data[compensated_pos*3 + i] = extension_cmd->ext/3.0;
                    i++; 
                }
                right_pub_.publish(right_array_);
            }

        };
        void pos_update(const sensor_msgs::JointStateConstPtr &state){
            float l_temp = fmod((*state).position[53], 2*M_PI);
            float r_temp = fmod((*state).position[53], 2*M_PI); //revert to 54 when temp cyl2 is reintroduced
            left_drive_pos_ = l_temp / (M_PI/4);
            right_drive_pos_ = r_temp / (M_PI/4);
        }
    public:
        Allocator(){
            extension_sub_ = nh_.subscribe<daedalus::Extension>("/daedalus/extend_rod", 20, &Allocator::extension_callback_, this);
            left_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/daedalus_left_controller/command", 10);
            right_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/daedalus_right_controller/command", 10);
            rod_pos_sub_=nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Allocator::pos_update, this);
            int i = 0;
            while (i<24){
                left_array_.data.push_back(0);
                right_array_.data.push_back(0);
                i++;
            }
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "allocator");
    Allocator allocator;
    ros::spin(); 
    return 0;
}