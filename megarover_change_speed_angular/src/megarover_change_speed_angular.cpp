#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class Megarover_Change_Speed_Angular
{
    public:
        Megarover_Change_Speed_Angular();

        double _linear_x;
        double _angular_z;

        void send_twist();

    private:
        ros::NodeHandle _nh;
        ros::Publisher _tw_pub;
};

Megarover_Change_Speed_Angular::Megarover_Change_Speed_Angular():
    _linear_x(0.0),
    _angular_z(0.0)
{
    _tw_pub = _nh.advertise<geometry_msgs::Twist>("rover_twist", 1);
}

void Megarover_Change_Speed_Angular::send_twist()
{
    geometry_msgs::Twist twist;

    twist.linear.x = _linear_x;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = _angular_z;

    ros::Rate loop_late(10);

    for (int i = 0; i < 10; i++){
        _tw_pub.publish(twist);
        ros::spinOnce();
        loop_late.sleep();
    }
}

int main(int argc, char** argv)
{
    if(argc != 3){
        printf("%s [speed (m/s)] [angular (rad/s)]", argv[0]);
        return 1;
    }
    
    ros::init(argc, argv, "megarover_change_speed_angular");
    Megarover_Change_Speed_Angular megarover_change_speed_angular;

    megarover_change_speed_angular._linear_x = std::atof(argv[1]);
    megarover_change_speed_angular._angular_z = std::atof(argv[2]);

    megarover_change_speed_angular.send_twist();
    
    return 0;
}