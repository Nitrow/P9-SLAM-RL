
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "std_msgs/String.h"


#define size 640



class lidar_modifier_class {

public:
    lidar_modifier_class();

    ~lidar_modifier_class();

private:
    ros::NodeHandle n;

    ros::Subscriber lasersub;
    ros::Publisher lidar_pub;
    void laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr &scan);
    sensor_msgs::LaserScan scanmodi;
    float ranges[size]{};
    float intensities[size]{};

};


    void lidar_modifier_class::laser_msg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan) {


        for (int i = 0; i < size; ++i) {

            if (scan->ranges[i] < scan->range_max) {
                ranges[i] = scan->ranges[i];

            }
            else {
                ranges[i] = scan->range_max;
            }

            intensities[i] = scan->intensities[i];

        }

        ros::Time scan_time = ros::Time::now();

        // Create a LaserScan Message
        scanmodi.scan_time = scan->scan_time;
        scanmodi.header.stamp = scan->header.stamp;
        scanmodi.header.frame_id = scan->header.frame_id;
        scanmodi.angle_min = scan->angle_min;
        scanmodi.angle_max = scan->angle_max;
        scanmodi.angle_increment = scan->angle_increment;
        scanmodi.time_increment = scan->time_increment;
        scanmodi.range_min = scan->range_min;
        scanmodi.range_max = scan->range_max;

        scanmodi.ranges.resize(size);
        scanmodi.intensities.resize(size);
        for(int i = 0; i < 640; ++i){
            scanmodi.ranges[i] = ranges[i];
            scanmodi.intensities[i] = intensities[i];
        }
        lidar_pub.publish(scanmodi);
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "sick_modifier");

    lidar_modifier_class lidar_modifier_class;

    ros::spin();


    return 0;

}

    lidar_modifier_class::lidar_modifier_class()
    {
        lasersub = n.subscribe("/lidar", 50, &lidar_modifier_class::laser_msg_Callback, this);
        lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan",50);

    }

lidar_modifier_class::~lidar_modifier_class() = default;
