#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "autoware_msgs/VehicleCmd.h"
#include <stdlib.h>
#include <cmdline.h>

#define LINEAR_X_LIMIT 0.5
#define ANGULAR_Z_LIMIT 1.0

class Autoware_to_Megarover
{
    public:
        Autoware_to_Megarover(double lim_linear, double lim_angular);

    private:
        ros::NodeHandle _nh;
        ros::Publisher _rover_twist;
        ros::Subscriber _vehicle_cmd;

        double lim_linear_;
        double lim_angular_;

        void ConvertTopic(const autoware_msgs::VehicleCmd::ConstPtr& command);
};

Autoware_to_Megarover::Autoware_to_Megarover(double lim_linear, double lim_angular)
    :   lim_linear_(LINEAR_X_LIMIT)
    ,   lim_angular_(ANGULAR_Z_LIMIT)
{
    /* 初期化 */
    lim_linear_ = lim_linear;
    lim_angular_ = lim_angular;
    _rover_twist = _nh.advertise<geometry_msgs::Twist>("rover_twist", 1);
    _vehicle_cmd = _nh.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1000, &Autoware_to_Megarover::ConvertTopic, this);
}

void Autoware_to_Megarover::ConvertTopic(const autoware_msgs::VehicleCmd::ConstPtr& command)
{
    geometry_msgs::Twist pub_twist;
    if(command->mode == 1){
        if (command->twist_cmd.twist.linear.x > lim_linear_){
            pub_twist.linear.x = lim_linear_;
        }
        else if (command->twist_cmd.twist.linear.x < -lim_linear_){
            pub_twist.linear.x = -lim_linear_;
        }
        else {
            pub_twist.linear.x  = command->twist_cmd.twist.linear.x;
        }
        pub_twist.linear.y  = 0.0;
        pub_twist.linear.z  = 0.0;
        pub_twist.angular.x = 0.0;
        pub_twist.angular.y = 0.0;
        if (command->twist_cmd.twist.angular.z > lim_angular_){
            pub_twist.angular.z = lim_angular_;
        }
        else if (command->twist_cmd.twist.angular.z < -lim_angular_){
            pub_twist.angular.z = -lim_angular_;
        }
        else {
            pub_twist.angular.z = command->twist_cmd.twist.angular.z;
        }
    }
    else{
        pub_twist.linear.x  = 0.0;
        pub_twist.linear.y  = 0.0;
        pub_twist.linear.z  = 0.0;
        pub_twist.angular.x = 0.0;
        pub_twist.angular.y = 0.0;
        pub_twist.angular.z = 0.0;
    }
    _rover_twist.publish(pub_twist);
}

int main(int argc, char** argv)
{
    cmdline::parser cmdparser;
    cmdparser.add<double>("lim_linear", 'l', "Upper limit of Linear speed",false, LINEAR_X_LIMIT);
    cmdparser.add<double>("lim_angular", 'a', "Upper limit of angular velocity", false, ANGULAR_Z_LIMIT);
    cmdparser.add("help", 'h', "Print Help");

    if (!cmdparser.parse(argc, argv) || cmdparser.exist("help")){
        std::cout << cmdparser.error_full() << cmdparser.usage();
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "autoware2megarover");
    Autoware_to_Megarover autoware2megarover(cmdparser.get<double>("lim_linear"), cmdparser.get<double>("lim_angular"));
    ros::spin();
    return 0;
}
