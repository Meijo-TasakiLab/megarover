#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

class Megarover_Rviz
{
    public:
        Megarover_Rviz();
        void Pose_Publish();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom;
        tf::TransformBroadcaster _broadcaster;
        tf::Transform transform;
        tf::Quaternion quaternion;
        
        void Odom_Subscribe(const nav_msgs::Odometry::ConstPtr& odom_msg);

};

Megarover_Rviz::Megarover_Rviz()
{
    _odom = _nh.subscribe<nav_msgs::Odometry>("rover_odo", 1000, &Megarover_Rviz::Odom_Subscribe, this);
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    quaternion.setValue(0.0, 0.0, 0.0, 1.0);
    transform.setRotation(quaternion);
}

void Megarover_Rviz::Pose_Publish()
{
    _broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
}

void Megarover_Rviz::Odom_Subscribe(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    transform.setOrigin(tf::Vector3(
       odom_msg->pose.pose.position.x,
       odom_msg->pose.pose.position.y,
       odom_msg->pose.pose.position.z 
    ));
    quaternion.setValue(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    transform.setRotation(quaternion);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "megarover_rviz");
    Megarover_Rviz megarover_rviz;
    ros::Rate loop_rate = 100;
    while (ros::ok())
    {
        megarover_rviz.Pose_Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}