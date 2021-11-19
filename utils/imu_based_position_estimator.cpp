#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "list"
#include "geometry_msgs/Vector3.h"
#include "cmath"

#define FILTER_ORDER 32
#define IMU_UPDATE_RATE 100.0

class Estimator{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber imu_sub;
        ros::Publisher xyz_pub;
        sensor_msgs::Imu latest_reading;
        std::list<float> cumulated_xaccel;
        std::list<float> cumulated_yaccel;
        std::list<float> cumulated_zaccel;
        geometry_msgs::Vector3 position;
        geometry_msgs::Vector3 velocity;
        geometry_msgs::Vector3 acceleration;
    public:
        Estimator(){
            int i=0;
            while (i<FILTER_ORDER){
                cumulated_xaccel.push_back(0);
                cumulated_yaccel.push_back(0);
                cumulated_zaccel.push_back(0);
                i++;
            }
            imu_sub=nh.subscribe("/daedalus/imu_0", FILTER_ORDER, &Estimator::imu_callback, this);
            xyz_pub=nh.advertise<geometry_msgs::Vector3>("/daedalus/position", FILTER_ORDER, false);
            position.x=0;
            position.y=0;
            position.z=0;
            velocity.x=0;
            velocity.y=0;
            velocity.z=0;
            acceleration.x=0;
            acceleration.y=0;
            acceleration.z=0;
        }
        void imu_callback(const sensor_msgs::ImuConstPtr& incoming_reading){
            latest_reading=*incoming_reading;
            cumulated_xaccel.push_front(latest_reading.linear_acceleration.x/FILTER_ORDER);
            cumulated_yaccel.push_front(latest_reading.linear_acceleration.y/FILTER_ORDER);
            cumulated_zaccel.push_front(latest_reading.linear_acceleration.z/FILTER_ORDER);
            acceleration.x-=cumulated_xaccel.back();
            acceleration.y-=cumulated_yaccel.back();
            acceleration.z-=cumulated_zaccel.back();
            cumulated_xaccel.pop_back();
            cumulated_yaccel.pop_back();
            cumulated_zaccel.pop_back();
            acceleration.x+=cumulated_xaccel.front();
            acceleration.y+=cumulated_yaccel.front();
            acceleration.z+=cumulated_zaccel.front();

            position.x+=(velocity.x*(1.0/IMU_UPDATE_RATE) + 0.5*acceleration.x*pow(1.0/IMU_UPDATE_RATE,2));
            position.y+=(velocity.y*(1.0/IMU_UPDATE_RATE) + 0.5*acceleration.y*pow(1.0/IMU_UPDATE_RATE,2));
            position.z+=(velocity.z*(1.0/IMU_UPDATE_RATE) + 0.5*acceleration.z*pow(1.0/IMU_UPDATE_RATE,2));
            velocity.x+=acceleration.x*(1.0/IMU_UPDATE_RATE);
            velocity.y+=acceleration.y*(1.0/IMU_UPDATE_RATE);
            velocity.z+=acceleration.z*(1.0/IMU_UPDATE_RATE);
            xyz_pub.publish(position);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_based_position_estimator");
    Estimator estimator;
    ros::spin();
    return 0;
}