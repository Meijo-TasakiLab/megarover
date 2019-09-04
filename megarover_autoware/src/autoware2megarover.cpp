#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "autoware_msgs/VehicleCmd.h"
#include <stdlib.h>

class Autoware_to_Megarover
{
    public:
        Autoware_to_Megarover();

    private:
        ros::NodeHandle _nh;
        ros::Publisher _rover_twist;
        ros::Subscriber _vehicle_cmd;
        char _sound_flag;

        void ConvertTopic(const autoware_msgs::VehicleCmd::ConstPtr& command);
};

Autoware_to_Megarover::Autoware_to_Megarover()
{
    _sound_flag = 0;
    _rover_twist = _nh.advertise<geometry_msgs::Twist>("rover_twist", 1);
    _vehicle_cmd = _nh.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1000, &Autoware_to_Megarover::ConvertTopic, this);
}

void Autoware_to_Megarover::ConvertTopic(const autoware_msgs::VehicleCmd::ConstPtr& command)
{
    geometry_msgs::Twist pub_twist;
    if(command->mode == 1){
        pub_twist.linear.x  = command->twist_cmd.twist.linear.x;
        pub_twist.linear.y  = 0.0;
        pub_twist.linear.z  = 0.0;
        pub_twist.angular.x = 0.0;
        pub_twist.angular.y = 0.0;
        pub_twist.angular.z = command->twist_cmd.twist.angular.z;
//        if(_sound_flag == 0){
//            system("mpv --loop-file ~/shared_dir/sound/megarover_sound.wav &");
//            _sound_flag = 1;
//        }
    }
    else{
        pub_twist.linear.x  = 0.0;
        pub_twist.linear.y  = 0.0;
        pub_twist.linear.z  = 0.0;
        pub_twist.angular.x = 0.0;
        pub_twist.angular.y = 0.0;
        pub_twist.angular.z = 0.0;
//        if(_sound_flag == 1){
//            system("killall mpv &");
//            _sound_flag = 0;
//        }
    }
    _rover_twist.publish(pub_twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autoware2megarover");
    Autoware_to_Megarover autoware2megarover;
    ros::spin();
    return 0;
}