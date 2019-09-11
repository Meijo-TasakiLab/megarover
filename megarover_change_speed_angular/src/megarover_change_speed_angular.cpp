#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <cmdline.h>

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

    /* Send Message */
    for (int i = 0; i < 10; i++){
        _tw_pub.publish(twist);
        ros::spinOnce();
        loop_late.sleep();
    }
}

int main(int argc, char** argv)
{
    /* コマンドラインパーサー */
    cmdline::parser cmdparser;
    cmdparser.add<double>("speed", 's', "Linear speed [m/s]", false, 0.0);
    cmdparser.add<double>("angular", 'a', "Angular velocity [rad/s]", false, 0.0);
    cmdparser.add("help", 'h', "Print Help");

    if (!cmdparser.parse(argc, argv) || cmdparser.exist("help")){
        std::cout << cmdparser.error_full() << cmdparser.usage();
        return EXIT_FAILURE;
    }

    /* ROSノード初期化 */
    ros::init(argc, argv, "megarover_change_speed_angular");
    Megarover_Change_Speed_Angular megarover_change_speed_angular;

    megarover_change_speed_angular._linear_x = cmdparser.get<double>("speed");
    megarover_change_speed_angular._angular_z = cmdparser.get<double>("angular");

    megarover_change_speed_angular.send_twist();
    
    return EXIT_SUCCESS;
}