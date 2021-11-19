#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Trigger.h"
#include "actionlib/server/simple_action_server.h"
#include "daedalus/stabilizeAction.h"

#define MAX_SWING_DIST 1
#define CTRL_UPDATE_RATE 5.0
#define ANG_VELS_CONSIDERED 10
#define ACCEPTABLE_ANG_VEL 5
#define ACCEPTABLE_ANG 10
#define ANGS_CONSIDERED 10

class rollStabilizer{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber activation_sub;
        ros::Publisher pendulum_pub;
        ros::Publisher pid_pub;
        actionlib::SimpleActionServer<daedalus::stabilizeAction> stabilizer_server;
        daedalus::stabilizeActionFeedback roll_derivative_feedback;
        boost::shared_ptr<const geometry_msgs::Vector3> latest_orientation;
        geometry_msgs::Vector3 previous_orientation;
        geometry_msgs::Vector3 controller_output;
        std_msgs::Float64 correction;
        float integrated_error;
        float previous_error;
        float error_derivative;
        float kp;
        float ki;
        float kd;
        float latest_roll_derivative;
    public:
        rollStabilizer():stabilizer_server(nh, "/daedalus/stabilize_roll_action", boost::bind(&rollStabilizer::stabilizer_callback, this, _1), false){
            pendulum_pub=nh.advertise<std_msgs::Float64>("/daedalus_battery_steer_controller/command", 1, false);
            pid_pub=nh.advertise<geometry_msgs::Vector3>("/daedalus/pid/roll", 1, false);
            stabilizer_server.start();
            integrated_error=0;
            previous_error=0;
            error_derivative=0;
            kp=0.4;
            ki=0;
            kd=0.8;
        }
        void stabilizer_callback(const daedalus::stabilizeGoalConstPtr &goal){
            float error;
            integrated_error=0;
            previous_error=0;
            error_derivative=0;
            ros::Rate rate(CTRL_UPDATE_RATE);
            while (ros::ok() && !stabilizer_server.isPreemptRequested()){
                latest_orientation=ros::topic::waitForMessage<geometry_msgs::Vector3>("/daedalus/euler_orientation");
                error=0-(*latest_orientation).x;
                controller_output.x=error*kp;
                integrated_error+=error*(1.0/CTRL_UPDATE_RATE);
                controller_output.y=integrated_error*ki;
                error_derivative=(error-previous_error)*CTRL_UPDATE_RATE;
                controller_output.z=error_derivative*kp;
                pid_pub.publish(controller_output);
                previous_error=error;
                error=kp*error+ki*integrated_error+kd*error_derivative;
                if(error>90)
                    error=90;
                else if(error<-90)
                    error=-90;
                correction.data=-1*(error/90)*MAX_SWING_DIST;
                pendulum_pub.publish(correction);
                latest_roll_derivative=((*latest_orientation).x - previous_orientation.x)*CTRL_UPDATE_RATE;
                previous_orientation=*latest_orientation;
                roll_derivative_feedback.feedback.feedback.push_back(latest_roll_derivative);
                rate.sleep();
            }
            stabilizer_server.setPreempted();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roll_stabilizer");
    rollStabilizer stabilizer;
    ros::spin(); 
    return 0;
}