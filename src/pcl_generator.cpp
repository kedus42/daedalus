#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "pcl_ros/transforms.h"
#include "Eigen/Dense"
#include "std_msgs/Bool.h"

#define AGGREGATE_UPDATE_RATE 20.0
#define AGGREGATE_MAP_UPDATE_RATE 60.0
#define LIDAR_THETA_OFFSET 7.5
#define LIDAR_X_OFFSET 0.1
#define LIDAR_Y_OFFSET 0.1
#define TF_TIMEOUT 2.0

class Translator {
     protected:
        ros::NodeHandle nh;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;
        ros::Publisher point_cloud_pub;
        ros::Publisher point_cloud_map_pub;
        ros::Subscriber scan0_sub;
        ros::Subscriber scan1_sub;
        ros::Subscriber scan2_sub;
        ros::Subscriber scan3_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber mapper_switch_sub;
        ros::Timer update_timer;
        ros::Timer feed_mapper_timer;
        sensor_msgs::LaserScan scan_0;
        sensor_msgs::LaserScan scan_1;
        sensor_msgs::LaserScan scan_2;
        sensor_msgs::LaserScan scan_3;
        sensor_msgs::PointCloud2 cloud_0;
        sensor_msgs::PointCloud2 cloud_1;
        sensor_msgs::PointCloud2 cloud_2;
        sensor_msgs::PointCloud2 cloud_3;
        boost::shared_ptr<const sensor_msgs::Imu> imu_shared_reading;
        sensor_msgs::Imu imu_reading;
        Eigen::Matrix4f orientation_matrix;
        Eigen::Matrix4f lidar_clockwise_offset_correction;
        Eigen::Matrix4f lidar_anticlockwise_offset_correction;
        Eigen::Quaternionf orientation;
        bool toggle_mapper;

     public:
        Translator(){
            tfListener_.waitForTransform("base_link", "anchor_0", ros::Time::now(), ros::Duration(TF_TIMEOUT));
            tfListener_.waitForTransform("base_link", "anchor_1", ros::Time::now(), ros::Duration(TF_TIMEOUT));
            tfListener_.waitForTransform("base_link", "anchor_2", ros::Time::now(), ros::Duration(TF_TIMEOUT));
            tfListener_.waitForTransform("base_link", "anchor_3", ros::Time::now(), ros::Duration(TF_TIMEOUT));
            update_timer=nh.createTimer(ros::Duration(1.0/AGGREGATE_UPDATE_RATE), &Translator::scanCallback, this);
            feed_mapper_timer=nh.createTimer(ros::Duration(1.0/AGGREGATE_MAP_UPDATE_RATE), &Translator::scanMapCallback, this);
            point_cloud_pub= nh.advertise<sensor_msgs::PointCloud2> ("/daedalus/cloud/aggregate", 100, false);
            point_cloud_map_pub= nh.advertise<sensor_msgs::PointCloud2> ("/daedalus/cloud/aggregate_map", 100, false);
            scan0_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/laser/scan_0", 1, &Translator::scan0_callback, this);
            scan1_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/laser/scan_1", 1, &Translator::scan1_callback, this);
            scan2_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/laser/scan_2", 1, &Translator::scan2_callback, this);
            scan3_sub = nh.subscribe<sensor_msgs::LaserScan>("/daedalus/laser/scan_3", 1, &Translator::scan3_callback, this);
            mapper_switch_sub = nh.subscribe<std_msgs::Bool>("/daedalus/switches/mapper", 1, &Translator::switch_callback, this);
            scan_0 = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/laser/scan_0", nh));
            scan_1 = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/laser/scan_1", nh));
            scan_2 = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/laser/scan_2", nh));
            scan_3 = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>("/daedalus/laser/scan_3", nh));
            imu_sub = nh.subscribe<sensor_msgs::Imu>("/daedalus/imu_0", 1, &Translator::imu_callback, this);
            orientation_matrix = Eigen::Matrix4f::Identity();
            lidar_clockwise_offset_correction = Eigen::Matrix4f::Identity();
            lidar_anticlockwise_offset_correction = Eigen::Matrix4f::Identity();
            lidar_clockwise_offset_correction(0,0) = cos(LIDAR_THETA_OFFSET*3.14/180);
            lidar_clockwise_offset_correction(0,1) = -1*sin(LIDAR_THETA_OFFSET*3.14/180);
            lidar_clockwise_offset_correction(1,0) = sin(LIDAR_THETA_OFFSET*3.14/180);
            lidar_clockwise_offset_correction(1,1) = cos(LIDAR_THETA_OFFSET*3.14/180);
            lidar_anticlockwise_offset_correction(0,0) = cos(-1*LIDAR_THETA_OFFSET*3.14/180);
            lidar_anticlockwise_offset_correction(0,1) = -1*sin(-1*LIDAR_THETA_OFFSET*3.14/180);
            lidar_anticlockwise_offset_correction(1,0) = sin(-1*LIDAR_THETA_OFFSET*3.14/180);
            lidar_anticlockwise_offset_correction(1,1) = cos(-1*LIDAR_THETA_OFFSET*3.14/180);
            toggle_mapper = true;
        }
        void scan0_callback(const sensor_msgs::LaserScanConstPtr &latest){
            scan_0=*latest;
        }
        void scan1_callback(const sensor_msgs::LaserScanConstPtr &latest){
            scan_1=*latest;
        }
        void scan2_callback(const sensor_msgs::LaserScanConstPtr &latest){
            scan_2=*latest;
        }
        void scan3_callback(const sensor_msgs::LaserScanConstPtr &latest){
            scan_3=*latest;
        }
        void imu_callback(const sensor_msgs::ImuConstPtr &latest){
            imu_reading = *latest;
            orientation.w()=imu_reading.orientation.w;
            orientation.x()=imu_reading.orientation.x;
            orientation.y()=imu_reading.orientation.y;
            orientation.z()=imu_reading.orientation.z;
            orientation.normalize();
            orientation_matrix.block(0,0,3,3)=orientation.toRotationMatrix();
        }
        void switch_callback(const std_msgs::BoolConstPtr &mapper_state){
            toggle_mapper = (*mapper_state).data;
        }
        void scanCallback(const ros::TimerEvent&){
            projector_.projectLaser(scan_0, cloud_0);
            projector_.projectLaser(scan_1, cloud_1);
            projector_.projectLaser(scan_2, cloud_2);
            projector_.projectLaser(scan_3, cloud_3);
            uint64_t OriginalSize0= cloud_0.data.size();
            uint64_t OriginalSize1= cloud_1.data.size();
            uint64_t OriginalSize2= cloud_2.data.size();
            pcl_ros::transformPointCloud("base_link", cloud_0, cloud_0, tfListener_);
            pcl_ros::transformPointCloud("base_link", cloud_1, cloud_1, tfListener_);
            pcl_ros::transformPointCloud("base_link", cloud_2, cloud_2, tfListener_);
            pcl_ros::transformPointCloud("base_link", cloud_3, cloud_3, tfListener_);
            pcl_ros::transformPointCloud(lidar_clockwise_offset_correction, cloud_0, cloud_0);
            pcl_ros::transformPointCloud(lidar_anticlockwise_offset_correction, cloud_1, cloud_1);
            pcl_ros::transformPointCloud(lidar_clockwise_offset_correction, cloud_2, cloud_2);
            pcl_ros::transformPointCloud(lidar_anticlockwise_offset_correction, cloud_3, cloud_3);

            cloud_0.width+=cloud_1.width+cloud_2.width+cloud_3.width;
            cloud_0.data.resize(cloud_0.data.size() + cloud_1.data.size() + cloud_2.data.size() + cloud_3.data.size());

            std::copy(cloud_1.data.begin(), cloud_1.data.end(), cloud_0.data.begin() + OriginalSize0);
            std::copy(cloud_2.data.begin(), cloud_2.data.end(), cloud_0.data.begin() + OriginalSize0+OriginalSize1);
            std::copy(cloud_3.data.begin(), cloud_3.data.end(), cloud_0.data.begin() + OriginalSize0+OriginalSize1+OriginalSize2);
            
            pcl_ros::transformPointCloud(orientation_matrix, cloud_0, cloud_0);

            point_cloud_pub.publish(cloud_0);
        }
        void scanMapCallback(const ros::TimerEvent&){
            if (toggle_mapper){
                projector_.projectLaser(scan_0, cloud_0);
                projector_.projectLaser(scan_1, cloud_1);
                projector_.projectLaser(scan_2, cloud_2);
                projector_.projectLaser(scan_3, cloud_3);
                uint64_t OriginalSize0= cloud_0.data.size();
                uint64_t OriginalSize1= cloud_1.data.size();
                uint64_t OriginalSize2= cloud_2.data.size();
                pcl_ros::transformPointCloud("base_link", cloud_0, cloud_0, tfListener_);
                pcl_ros::transformPointCloud("base_link", cloud_1, cloud_1, tfListener_);
                pcl_ros::transformPointCloud("base_link", cloud_2, cloud_2, tfListener_);
                pcl_ros::transformPointCloud("base_link", cloud_3, cloud_3, tfListener_);
                pcl_ros::transformPointCloud(lidar_clockwise_offset_correction, cloud_0, cloud_0);
                pcl_ros::transformPointCloud(lidar_anticlockwise_offset_correction, cloud_1, cloud_1);
                pcl_ros::transformPointCloud(lidar_clockwise_offset_correction, cloud_2, cloud_2);
                pcl_ros::transformPointCloud(lidar_anticlockwise_offset_correction, cloud_3, cloud_3);

                cloud_0.width+=cloud_1.width+cloud_2.width+cloud_3.width;
                cloud_0.data.resize(cloud_0.data.size() + cloud_1.data.size() + cloud_2.data.size() + cloud_3.data.size());

                std::copy(cloud_1.data.begin(), cloud_1.data.end(), cloud_0.data.begin() + OriginalSize0);
                std::copy(cloud_2.data.begin(), cloud_2.data.end(), cloud_0.data.begin() + OriginalSize0+OriginalSize1);
                std::copy(cloud_3.data.begin(), cloud_3.data.end(), cloud_0.data.begin() + OriginalSize0+OriginalSize1+OriginalSize2);
                
                pcl_ros::transformPointCloud(orientation_matrix, cloud_0, cloud_0);

                point_cloud_map_pub.publish(cloud_0);
            }
            
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_generator");
    Translator translator;
    ros::spin();
    return 0;
}