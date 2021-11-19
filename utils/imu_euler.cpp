#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "math.h"

class quat_to_euler{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber quat_sub;
        ros::Publisher euler_pub;
        geometry_msgs::Vector3 latest_rpy_publishable;
        double latest_rpy[3];
    public:
        quat_to_euler(){
            quat_sub=nh.subscribe("/daedalus/imu_0", 10, &quat_to_euler::quat_callback, this);
            euler_pub=nh.advertise<geometry_msgs::Vector3>("/daedalus/euler_orientation", 10, false);
            latest_rpy_publishable.x=0;
            latest_rpy_publishable.y=0;
            latest_rpy_publishable.z=0;
        }
        void quat_callback(const sensor_msgs::Imu &raw_imu){
            convert_to_euler(raw_imu.orientation, latest_rpy);
            latest_rpy_publishable.x=latest_rpy[0]*180/M_PI;
            latest_rpy_publishable.y=latest_rpy[1]*180/M_PI;
            latest_rpy_publishable.z=latest_rpy[2]*180/M_PI;
            euler_pub.publish(latest_rpy_publishable);
        }
        void convert_to_euler(geometry_msgs::Quaternion quaternion, double rpy[3]){
            tf::Quaternion quat;
            tf::quaternionMsgToTF(quaternion, quat);
            tf::Matrix3x3(quat).getRPY(rpy[0], rpy[1], rpy[2]);
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_euler");
    quat_to_euler translator;
    ros::spin();
    return 0;
}