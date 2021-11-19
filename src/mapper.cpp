#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "laser_assembler/AssembleScans2.h"
#include "Eigen/Dense"
#include "pcl_ros/transforms.h"
#include "std_msgs/Bool.h"
#include "pcl/registration/icp.h"
#include "pcl_ros/point_cloud.h"

class Mapper{
    protected: 
        ros::ServiceClient assembler_client;
        sensor_msgs::PointCloud2 latest_cloud;
        sensor_msgs::PointCloud2 corrected_cloud;
        laser_assembler::AssembleScans2 srv;
        pcl::PointCloud<pcl::PointXYZ> latest_cloud_pcl;
        pcl::PointCloud<pcl::PointXYZ> corrected_cloud_pcl;
        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Subscriber pose_sub;  
        ros::Subscriber switch_sub;
        ros::Subscriber drive_sub;
        ros::Publisher map_pub;
        ros::Publisher map_corrected_pub;
        Eigen::Matrix4f position_matrix;
        Eigen::Matrix4f icp_correction;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> latest_shared;
        pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> corrected_shared;
        bool toggle_mapper;
        bool registration_started;
    public:
        Mapper(){
            cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/daedalus/cloud/aggregate_map", 1, &Mapper::cloud_callback, this);
            pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/daedalus/pose_estimate", 1, &Mapper::pose_callback, this);
            switch_sub = nh.subscribe<std_msgs::Bool>("/daedalus/switches/mapper", 1, &Mapper::mapper_switch, this);
            drive_sub = nh.subscribe<std_msgs::Float64MultiArray>("/daedalus_battery_velocity_controller/command", 1, &Mapper::drive_callback, this);
            map_pub = nh.advertise<sensor_msgs::PointCloud2>("/daedalus/map", 1, false);
            map_corrected_pub = nh.advertise<sensor_msgs::PointCloud2>("/daedalus/corrected_map", 1, false);
            position_matrix = Eigen::Matrix4f::Identity();
            ros::service::waitForService("assemble_scans_mapping");
            assembler_client=nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2_mapper");
            srv.request.begin=ros::Time::now();
            icp_correction = Eigen::Matrix4f::Identity();
            icp.setMaxCorrespondenceDistance(0.05);
            icp.setMaximumIterations(150);
            icp.setTransformationEpsilon (1e-12);
            icp.setEuclideanFitnessEpsilon(0.1);
            toggle_mapper = true;
            registration_started = false;
        }
        void mapper_switch(const std_msgs::BoolConstPtr &state){
            toggle_mapper = (*state).data;
        }
        void pose_callback(const geometry_msgs::PoseStampedConstPtr &pose){
            position_matrix(0,3) = (*pose).pose.position.x;
            position_matrix(1,3) = (*pose).pose.position.y;
            //position_matrix(2,3) = (*pose).pose.position.z;
        }
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud){
            if (toggle_mapper){
                pcl_ros::transformPointCloud(position_matrix, *cloud, latest_cloud);
                map_pub.publish(latest_cloud);
            }
        }
        void drive_callback(const std_msgs::Float64MultiArrayConstPtr &drive_command){
            if ((*drive_command).data[0]!=0){
                if(registration_started){
                    srv.request.end = ros::Time::now();
                    assembler_client.call(srv);
                    srv.response.cloud.header.frame_id = "base_link";
                    pcl::fromROSMsg(srv.response.cloud, latest_cloud_pcl);
                    pcl::fromROSMsg(corrected_cloud, corrected_cloud_pcl);
                    corrected_shared = corrected_cloud_pcl.makeShared();
                    latest_shared = latest_cloud_pcl.makeShared();
                    icp.setInputTarget(corrected_shared);
                    icp.setInputSource(latest_shared);
                    icp.align(latest_cloud_pcl);
                    icp_correction = icp.getFinalTransformation();
                    pcl_ros::transformPointCloud(icp_correction, srv.response.cloud, srv.response.cloud);
                    map_corrected_pub.publish(srv.response.cloud);
                    corrected_cloud = srv.response.cloud;
                } else{
                    srv.request.end = ros::Time::now();
                    assembler_client.call(srv);
                    srv.response.cloud.header.frame_id = "base_link";
                    map_corrected_pub.publish(srv.response.cloud);
                    corrected_cloud  = srv.response.cloud;
                    registration_started = true;
                }
            } else{
                srv.request.begin = ros::Time::now();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapper");
    Mapper mapper;
    ros::spin(); 
    return 0;
}