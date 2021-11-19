#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/PoseStamped.h"
#include "laser_assembler/AssembleScans2.h"
#include "Eigen/Dense"
#include "pcl_ros/transforms.h"

class Mapper{
    protected: 
        ros::ServiceClient assembler_client;
        sensor_msgs::PointCloud2 latest_cloud;
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
        message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;  
        message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> scan_pose_synch;
        ros::Publisher map_pub;
        Eigen::Matrix4f position_matrix;
    public:
        Mapper():cloud_sub(nh, "/daedalus/cloud/aggregate_map", 1), pose_sub(nh, "/daedalus/pose_estimate", 1), scan_pose_synch(cloud_sub, pose_sub, 1){
            scan_pose_synch.registerCallback(boost::bind(&Mapper::mapping_callback, this, _1, _2));
            map_pub = nh.advertise<sensor_msgs::PointCloud2>("/daedalus/map", 1, false);
            position_matrix = Eigen::Matrix4f::Identity();
        }
        void mapping_callback(const sensor_msgs::PointCloud2ConstPtr &cloud, const geometry_msgs::PoseStampedConstPtr &pose){
            position_matrix(0,3) = (*pose).pose.position.x;
            position_matrix(1,3) = (*pose).pose.position.y;
            latest_cloud = *cloud;
            pcl_ros::transformPointCloud(position_matrix, latest_cloud, latest_cloud);
            map_pub.publish(latest_cloud);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapper");
    Mapper mapper;
    ros::spin(); 
    return 0;
}