#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

class Megarover_to_Autoware
{
public:
  Megarover_to_Autoware(char pose_en, char odom_en, char* sound_path);
  void Main();

private:
  ros::NodeHandle _nh;
  ros::Publisher _current_pose;
  ros::Publisher _current_velocity;
  ros::Publisher _vehicle_odom;
  ros::Subscriber _rover_odom;

  double x;
  double y;
  double th;
  double vx;
  double vy;
  double vth;

  char mode_cp;
  char mode_vo;
  char sound_flag;
  char sound_cmd[];

  void roverOdomCallback(const geometry_msgs::Twist::ConstPtr &rover_odom);
};

Megarover_to_Autoware::Megarover_to_Autoware(char pose_en, char odom_en, char* sound_path)
{
  if(pose_en == '1'){
    mode_cp = 1;
    _current_pose = _nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
  }
  else{
    mode_cp  = 0;
  }

  if(odom_en == '1'){
    mode_vo = 1;
    _vehicle_odom = _nh.advertise<nav_msgs::Odometry>("/vehicle/odom",1);
  }
  else{
    mode_vo = 0;
  }
  sound_flag = 0;
  printf("%s\n", sound_path);
  sprintf(sound_cmd, "%s %s %s", "mpg321", sound_path, "-l 0 &");
  printf("%s\n", sound_cmd);
  _current_velocity = _nh.advertise<geometry_msgs::TwistStamped>("/current_velocity", 1);
  _rover_odom = _nh.subscribe<geometry_msgs::Twist>("/rover_odo", 1000, &Megarover_to_Autoware::roverOdomCallback, this);
}

void Megarover_to_Autoware::Main()
{
  double dt;
  double dx;
  double dy;
  double dth;
  x = 0.0;
  y = 0.0;
  th = 0.0;
  vx = 0.0;
  vy = 0.0;
  vth = 0.0;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate rate(20);
  while (_nh.ok())
  {
    ros::spinOnce();
    current_time = ros::Time::now();

    dt = (current_time - last_time).toSec();
    dx = (vx * cos(th) - vy * sin(th)) * dt;
    dy = (vx * sin(th) + vy * cos(th)) * dt;
    dth = vth * dt;

    x += dx;
    y += dy;
    th += dth;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    if (odom_quat.x == 0.0 && odom_quat.y == 0.0 && odom_quat.z == 0.0 && odom_quat.w == 0.0)
    {
      odom_quat.w = 1.0;
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped velocity;
    nav_msgs::Odometry odom;

    pose.header.stamp = current_time;
    pose.header.frame_id = "/map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = odom_quat;

    velocity.header.stamp = current_time;
    velocity.header.frame_id = "/base_link";
    velocity.twist.linear.x = vx;
    velocity.twist.linear.y = vy;
    velocity.twist.angular.z = vth;

    odom.header.stamp = current_time;
    odom.header.frame_id = "/map";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    _current_velocity.publish(velocity);
    if(mode_cp > 0) _current_pose.publish(pose);
    if(mode_vo > 0) _vehicle_odom.publish(odom);

    last_time = current_time;
    ros::spinOnce();
    rate.sleep();
  }
}

void Megarover_to_Autoware::roverOdomCallback(const geometry_msgs::Twist::ConstPtr &rover_odom)
{
  vx = rover_odom->linear.x;
  vth = rover_odom->angular.z;

  if(fabs(vx) > 0.01 || fabs(vth) > 0.01){
    if(sound_flag < 1){
      system(sound_cmd);
      sound_flag = 10;
    }
  }
  else{
    if(sound_flag > 0){
      sound_flag--;
      if(sound_flag < 1){
        system("killall mpg321 &");
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "megarover2autoware");
  if(argc != 4){
    printf("\n");
    printf("%s [/current_pose] [/vehicle/odom] [sound file]\n", argv[0]);
    printf("[/current_pose] <- on:1, off:0\n");
    printf("[/vehicle/odom] <- on:1, off:0\n");
    printf("[sound file]    <- Path to Audio File (.wav)\n");
    return 1;
  }
  Megarover_to_Autoware megarover2autoware(argv[1][0], argv[2][0], argv[3]);
  megarover2autoware.Main();
  return 0;
}
