#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "vehicle_blue/odom";
    static_transformStamped.child_frame_id = "base_link";
    static_transformStamped.transform.translation.x = msg->pose.pose.position.x;
    static_transformStamped.transform.translation.y = msg->pose.pose.position.y;

    static_transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    static_transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    static_broadcaster.sendTransform(static_transformStamped);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_tf_transform");
	ros::NodeHandle n;//create a handle node

	ros::Subscriber sub = n.subscribe("/model/vehicle_blue/odometry",10, chatterCallback);
        //subscriber lister the msgs of given topic
        ros::spin();//run it until you press ctrl+c
	return 0;
}
