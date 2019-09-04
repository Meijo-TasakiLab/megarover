#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Megarover_Receive_Speed_Angular
{
  public:
    Megarover_Receive_Speed_Angular();

    void _receive_twist(const geometry_msgs::Twist::ConstPtr& twist);
    double get_speed();
    double get_angular();

  private:
    double _linear_x;
    double _angular_z;

    ros::NodeHandle _nh;
    ros::Subscriber _tw_sub;
};

Megarover_Receive_Speed_Angular::Megarover_Receive_Speed_Angular()
{
  _tw_sub = _nh.subscribe<geometry_msgs::Twist>("rover_odo", 1000, &Megarover_Receive_Speed_Angular::_receive_twist, this);
}

void Megarover_Receive_Speed_Angular::_receive_twist(const geometry_msgs::Twist::ConstPtr& twist)
{
  _linear_x = twist->linear.x;
  _angular_z = twist->angular.z;
  printf("x軸方向の並進移動量：%f [m/s]\n",_linear_x);
  printf("z軸周りの旋回量　　：%f [rad/s]\n\n",_angular_z);
}

double Megarover_Receive_Speed_Angular::get_speed()
{
  return _linear_x;
}

double Megarover_Receive_Speed_Angular::get_angular()
{
  return _angular_z;
}

int main(int argc, char **argv)
{
  // ROSの初期化　ノードの名前特定
  ros::init(argc, argv, "megarover_receive_speed_angular");

  Megarover_Receive_Speed_Angular megarover_receive_speed_angular;

  ros::spin();

  return 0;
}
