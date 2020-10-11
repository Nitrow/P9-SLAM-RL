
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

            if (scan->ranges[i] < 10) {
                ranges[i] = scan->ranges[i];

            }
            else {
                ranges[i] = 10;
            }

            intensities[i] = scan->intensities[i];

        }

        ros::Time scan_time = ros::Time::now();

        // Create a LaserScan Message
        scanmodi.header.stamp = scan_time;
        scanmodi.header.frame_id = "vehicle_blue/lidar/gpu_lidar";
        scanmodi.angle_min = -3.14;
        scanmodi.angle_max = 3.14;
        scanmodi.angle_increment = 0.009827855974435806;
        scanmodi.time_increment = 0.0;
        scanmodi.range_min = 0.08;
        scanmodi.range_max = 10;

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
        lasersub = n.subscribe("/lidar", 1, &lidar_modifier_class::laser_msg_Callback, this);
        lidar_pub = n.advertise<sensor_msgs::LaserScan>("/scan",50);

    }

lidar_modifier_class::~lidar_modifier_class() = default;
