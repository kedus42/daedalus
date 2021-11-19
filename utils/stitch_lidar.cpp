#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud.h>
using namespace laser_assembler;

#define MAX_SCANS 120
#define LIDAR_UPDATE_RATE 60

class Sticher{
    protected:
        ros::NodeHandle nh;
    public:
        Sticher(){
        }
        void scans_callback(const ros::TimerEvent&){
        }
};

