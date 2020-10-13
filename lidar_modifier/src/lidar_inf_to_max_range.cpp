
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "std_msgs/String.h"


class lidar_modifier_class {

public:
    lidar_modifier_class();

    ~lidar_modifier_class();

private:
    ros::NodeHandle n;

    ros::Subscriber ladersub;
    ros::Publisher lidar_pub;
    sensor_msgs::LaserScan scan_modified;

    void laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr &scan);
};

void lidar_modifier_class::laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr &scan) {

    // Create a LaserScan Message
    scan_modified.scan_time = scan->scan_time;
    scan_modified.header.stamp = scan->header.stamp;
    scan_modified.header.frame_id = scan->header.frame_id;
    scan_modified.angle_min = scan->angle_min;
    scan_modified.angle_max = scan->angle_max;
    scan_modified.angle_increment = scan->angle_increment;
    scan_modified.time_increment = scan->time_increment;
    scan_modified.range_min = scan->range_min;
    scan_modified.range_max = scan->range_max;

    scan_modified.ranges.resize(scan->ranges.size());
    scan_modified.intensities.resize(scan->intensities.size());
    for (int i = 0; i < scan->ranges.size(); ++i) {

        if (scan->ranges[i] <= scan->range_max) {
            scan_modified.ranges[i] = scan->ranges[i];
        } else {
            scan_modified.ranges[i] = scan->range_max;
        }
        scan_modified.intensities[i] = scan->intensities[i];
    }
    lidar_pub.publish(scan_modified);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sick_modifier");

    lidar_modifier_class lidar_modifier_class;

    ros::spin();
    return 0;
}

lidar_modifier_class::lidar_modifier_class() {
    ladersub = n.subscribe("/lidar", 50, &lidar_modifier_class::laser_msg_Callback, this);
    lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 50);

}

lidar_modifier_class::~lidar_modifier_class() = default;
