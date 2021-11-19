#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "laser_assembler/AssembleScans2.h"
#include "std_msgs/Bool.h"

class Concatenator{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber trigger_sub;
        ros::Publisher concatenated_pub;
        laser_assembler::AssembleScans2 srv;
        ros::ServiceClient assembler_client;
    public:
        Concatenator(){
            trigger_sub = nh.subscribe<std_msgs::Bool>("/daedalus/concat_map_trigger",  1, &Concatenator::trigger_callback, this);
            concatenated_pub = nh.advertise<sensor_msgs::PointCloud2>("/daedalus/concatenated_map", 1, true);
            ros::service::waitForService("assemble_scans");
            assembler_client=nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
            srv.request.begin = ros::Time(0,0);
        }
        void trigger_callback(const std_msgs::BoolConstPtr &){
            srv.request.end=ros::Time::now();
            assembler_client.call(srv);
            concatenated_pub.publish(srv.response.cloud);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_concatenator");
    Concatenator concatenator;
    ros::spin(); 
    return 0;
} 