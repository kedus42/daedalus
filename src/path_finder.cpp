#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "laser_assembler/AssembleScans2.h"
#include "boost/foreach.hpp"
#include "pcl/octree/octree.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "vector"
#include "asa058/asa058.hpp"
#include "geometry_msgs/Vector3.h"
#include "actionlib/client/simple_action_client.h"
#include "daedalus/stabilizeAction.h"
#include "std_msgs/Bool.h"
#include "cstdlib"
#include "pcl_ros/transforms.h"

#define EXPLORE_DURATION 20
#define SCAN_DURATION 7.5
#define STABILIZATION_WAIT 0
#define CLOCK_INIT_WAIT 3
#define TUNNEL_WIDTH 4
#define MAX_STEER_ANGLE 60
#define ACCEPTABLE_TARGET_ERROR 10
#define YAW_CTRL_KP 1.0
#define ACCEPTABLE_SCAN_PITCH -45 

class pathFinder{
    private:
        std_msgs::Float64 target;
        std_msgs::Float64MultiArray fwd_command;
        std_msgs::Float64MultiArray bwd_command;
        std_msgs::Float64MultiArray stop_command;
        boost::shared_ptr<const geometry_msgs::Vector3> latest_orientation;
        sensor_msgs::PointCloud latest_cloud;
        std_msgs::Bool toggle_pitch_spin_controllers;
        laser_assembler::AssembleScans2 srv;
        geometry_msgs::Vector3 yaw_pid;
        ros::NodeHandle nh;
        ros::Publisher target_pub;
        ros::Publisher drive_pub;
        ros::Publisher stitched_pub;
        ros::Publisher pitch_switch_pub;
        ros::Publisher spin_switch_pub;
        ros::Publisher mapper_switch_pub;
        ros::Timer exploration_timer;
        ros::Timer scan_timer;
        ros::ServiceClient assembler_client;
        actionlib::SimpleActionClient<daedalus::stabilizeAction> stabilizer_client;
        daedalus::stabilizeGoal goal;
        Eigen::Matrix4f rotation_matrix;

    public:
        pathFinder():stabilizer_client("/daedalus/stabilize_roll_action", true){
            ros::Duration(CLOCK_INIT_WAIT).sleep();
            target.data=0;
            fwd_command.data.push_back(40);
            fwd_command.data.push_back(40);
            bwd_command.data.push_back(-40);
            bwd_command.data.push_back(-40);
            stop_command.data.push_back(0);
            stop_command.data.push_back(0);
            target_pub=nh.advertise<std_msgs::Float64>("/daedalus/target", 1, true);
            stitched_pub=nh.advertise<sensor_msgs::PointCloud2>("/daedalus/cloud/stitched", 1);
            drive_pub=nh.advertise<std_msgs::Float64MultiArray>("/daedalus_battery_velocity_controller/command", 1, true);
            pitch_switch_pub=nh.advertise<std_msgs::Bool>("/daedalus/switches/pitch_control", 1, true);
            spin_switch_pub=nh.advertise<std_msgs::Bool>("/daedalus/switches/spin_control", 1, true);
            mapper_switch_pub=nh.advertise<std_msgs::Bool>("/daedalus/switches/mapper", 1, true);
            ros::service::waitForService("assemble_scans");
            assembler_client=nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
            stabilizer_client.waitForServer();
            rotation_matrix=Eigen::Matrix4f::Identity(4,4);
            srv.request.begin = ros::Time::now();
            scan_timer=nh.createTimer(ros::Duration(SCAN_DURATION), &pathFinder::scans_callback, this, true, false);
            exploration_timer=nh.createTimer(ros::Duration(EXPLORE_DURATION), &pathFinder::pause_exploration, this, true, false);
            scan_timer.setPeriod(ros::Duration(SCAN_DURATION), true);
            scan_timer.start();
            stabilizer_client.sendGoal(goal);
        }
        void scans_callback(const ros::TimerEvent&){
            ROS_INFO("scan timer called");
            srv.request.end=ros::Time::now();
            assembler_client.call(srv);
            stitched_pub.publish(srv.response.cloud);

            target.data=retrieve_target_2means(srv.response.cloud);
            
            toggle_pitch_spin_controllers.data=false;
            spin_switch_pub.publish(toggle_pitch_spin_controllers);
            pitch_switch_pub.publish(toggle_pitch_spin_controllers);
            mapper_switch_pub.publish(toggle_pitch_spin_controllers);
            stabilizer_client.cancelGoal();
            ros::param::set("/controller_on", true);
            target_pub.publish(target);
            drive_pub.publish(fwd_command);
            exploration_timer.setPeriod(ros::Duration(EXPLORE_DURATION), true);
            exploration_timer.start();
        }
        void pause_exploration(const ros::TimerEvent&){
            ROS_INFO("exploration timer called");
            yaw_pid = *ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/pid/yaw", nh);
            float yaw_error = yaw_pid.x/YAW_CTRL_KP;
            ROS_INFO("error remaining in yaw: %f", yaw_error);
            ros::Rate rate(2);
            while(ros::ok() && (abs(yaw_error)>ACCEPTABLE_TARGET_ERROR || (*latest_orientation).y<ACCEPTABLE_SCAN_PITCH)){
                yaw_pid = *ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/pid/yaw", nh);
                latest_orientation=ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
                yaw_error = yaw_pid.x/YAW_CTRL_KP;
                ROS_INFO("error remaining in yaw: %f", yaw_error);
                ROS_INFO("current pitch: %f", (*latest_orientation).y);
                rate.sleep();
            }
            drive_pub.publish(stop_command);
            stabilizer_client.sendGoal(goal);
            ros::param::set("/controller_on", false);
            toggle_pitch_spin_controllers.data=true;
            spin_switch_pub.publish(toggle_pitch_spin_controllers);
            pitch_switch_pub.publish(toggle_pitch_spin_controllers);
            mapper_switch_pub.publish(toggle_pitch_spin_controllers);
            ros::Duration(STABILIZATION_WAIT).sleep();
            srv.request.begin = ros::Time::now();
            scan_timer.setPeriod(ros::Duration(SCAN_DURATION), true);
            scan_timer.start();
        }
        float retrieve_target_2means(sensor_msgs::PointCloud2 pcl2_cloud){
            int iterator=0;
            int point_count=0;
            float yaw_reference;
            std::vector<double> y_coordinates_vec;
            latest_orientation=ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
            yaw_reference = (*latest_orientation).z;
            rotation_matrix(0,0)=cos((-1*(*latest_orientation).z)*3.14/180);
            rotation_matrix(0,1)=-sin((-1*(*latest_orientation).z)*3.14/180);
            rotation_matrix(1,0)=sin((-1*(*latest_orientation).z)*3.14/180);
            rotation_matrix(1,1)=cos((-1*(*latest_orientation).z)*3.14/180);
            pcl_ros::transformPointCloud(rotation_matrix, pcl2_cloud, pcl2_cloud);
            sensor_msgs::convertPointCloud2ToPointCloud(pcl2_cloud, latest_cloud);
            sensor_msgs::PointCloud pcl1;
            BOOST_FOREACH (const geometry_msgs::Point32 pt, latest_cloud.points){
                if(pt.x>0 && !(pt.x<0.2 && pt.z<0)){
                    y_coordinates_vec.push_back(pt.y);
                    pcl1.points.push_back(pt);
                    point_count++;
                }
                iterator++;
            }
            sensor_msgs::PointCloud2 pcl2;
            sensor_msgs::convertPointCloudToPointCloud2(pcl1, pcl2);
            pcl2.header=pcl2_cloud.header;
            stitched_pub.publish(pcl2);
            double centers[] = {-2, 2};
            double* y_coordinates_arr = &y_coordinates_vec[0];
            double sum_of_sqdev[] ={0,0};
            double workspace[point_count];
            int assignments[point_count];
            int cluster_sizes[] = {0,0};
            clustr(y_coordinates_arr, centers, sum_of_sqdev, assignments, workspace, cluster_sizes, point_count, 1, 2, 300, 2);
            ROS_INFO("centers: %f, %f", centers[0], centers[1]);
            float furthest_wall_ycoordinate=centers[0];
            if (abs(centers[1]) >= abs(centers[0]))
                furthest_wall_ycoordinate = centers[1];
            float computed_target = (((centers[0]+centers[1])/2)/abs(furthest_wall_ycoordinate))*MAX_STEER_ANGLE;
            latest_orientation=ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation", nh);
            float spin_correction = yaw_reference -(*latest_orientation).z;
            ROS_INFO("computed target: %f", computed_target);
            ROS_INFO("spin correction: %f", spin_correction);
            ROS_INFO("target after correction: %f", spin_correction + computed_target);
            return computed_target + spin_correction;
        }
        float find_point_elligibility(float x, float y, float theta){
            return x*cos(theta*3.14/180.0) + y*sin(theta*3.14/180.0);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_finder");
    pathFinder finder;
    ros::spin(); 
    return 0;
}