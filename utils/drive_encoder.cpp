#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int64.h"

#define TICK_THRESHOLD 0.1
#define UPDATE_RATE 10.0
#define DIST_TO_RODS 0.1

class Encoder{
    protected:
        int right_ticks;
        int left_ticks;
        std_msgs::Int64 left_pos;
        std_msgs::Int64 right_pos;
        sensor_msgs::LaserScan right_prev_reading;
        sensor_msgs::LaserScan left_prev_reading;
        ros::NodeHandle nh;
        ros::Publisher right_pos_pub;
        ros::Publisher left_pos_pub;
        ros::Subscriber right_encoder_sub;
        ros::Subscriber left_encoder_sub;
        ros::Timer update_timer;
    public:
        Encoder(){
            right_ticks=0;
            left_ticks=0;
            left_pos.data=0;
            right_pos.data=0;
            right_pos_pub = nh.advertise<std_msgs::Int64>("/daedalus/right_drive_pos", 1, true);
            left_pos_pub = nh.advertise<std_msgs::Int64>("/daedalus/left_drive_pos", 1, true);
            right_encoder_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/drive_encoder_r", 10, &Encoder::right_callback, this);
            left_encoder_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/drive_encoder_l", 10, &Encoder::left_callback, this);
            right_prev_reading = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/drive_encoder_r", nh);
            left_prev_reading = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/drive_encoder_l", nh);
            update_timer = nh.createTimer(ros::Duration(1.0/UPDATE_RATE), &Encoder::update, this);
        }
        void right_callback(const sensor_msgs::LaserScanConstPtr &right_reading){
            if ((*right_reading).ranges[0]>DIST_TO_RODS && right_prev_reading.ranges[0]<DIST_TO_RODS){
                right_ticks+=1;
            }
            if ((*right_reading).ranges[1]>DIST_TO_RODS && right_prev_reading.ranges[1]<DIST_TO_RODS){
                right_ticks+=1;
            }
            if ((*right_reading).ranges[2]>DIST_TO_RODS && right_prev_reading.ranges[2]<DIST_TO_RODS){
                right_ticks+=1;
            }
            if (right_ticks>=3){
                right_ticks=0;
                if (right_pos.data<7)
                    right_pos.data+=1;
                else
                    right_pos.data=0;
                ROS_INFO("right drive pos: %ld", right_pos.data);
            }
            right_prev_reading = *right_reading;
        }
        void left_callback(const sensor_msgs::LaserScanConstPtr &left_reading){
            if ((*left_reading).ranges[0]>DIST_TO_RODS && left_prev_reading.ranges[0]<DIST_TO_RODS){
                left_ticks+=1;
            }
            if ((*left_reading).ranges[1]>DIST_TO_RODS && left_prev_reading.ranges[1]<DIST_TO_RODS){
                left_ticks+=1;
            }
            if ((*left_reading).ranges[2]>DIST_TO_RODS && left_prev_reading.ranges[2]<DIST_TO_RODS){
                left_ticks+=1;
            }
            if (left_ticks>=3){
                left_ticks=0;
                if (left_pos.data<7)
                    left_pos.data+=1;
                else
                    left_pos.data=0;
                ROS_INFO("left drive pos: %ld", left_pos.data);
            }
            left_prev_reading = *left_reading;
        }
        void update(const ros::TimerEvent &){
            left_pos_pub.publish(left_pos);
            right_pos_pub.publish(right_pos);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "drive_encoder");
    Encoder encoder;
    ros::spin();
    return 0;
}