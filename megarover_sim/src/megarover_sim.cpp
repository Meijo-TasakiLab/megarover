#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class MegaroverSim
{
    public:
        MegaroverSim();

    private:
        ros::NodeHandle _nh;
        ros::Publisher _rover_odo;
        ros::Subscriber _rover_twist;

        void ConvertTopic(const geometry_msgs::Twist::ConstPtr& rover_twist);
};

MegaroverSim::MegaroverSim()
{
    _rover_odo = _nh.advertise<geometry_msgs::Twist>("rover_odo", 1);
    _rover_twist = _nh.subscribe<geometry_msgs::Twist>("rover_twist", 1000, &MegaroverSim::ConvertTopic, this);
}

void MegaroverSim::ConvertTopic(const geometry_msgs::Twist::ConstPtr& rover_twist)
{
    geometry_msgs::Twist current_twist;
    current_twist.linear.x = rover_twist->linear.x;
    current_twist.linear.y = rover_twist->linear.y;
    current_twist.linear.z = rover_twist->linear.z;
    current_twist.angular.x = rover_twist->angular.x;
    current_twist.angular.y = rover_twist->angular.y;
    current_twist.angular.z = rover_twist->angular.z;
    _rover_odo.publish(current_twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "megarover_sim");
    MegaroverSim megarover_sim;
    ros::spin();
    return 0;
}