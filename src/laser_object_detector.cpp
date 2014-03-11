#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "youbot_pick_and_place/ObjectLocation.h"

#define RANGE_X 1
#define RANGE_Y 0.2

class Detector {
    public:
        //Detector():node("~") {
        Detector():node() {
            //pub = node.advertise<sensor_msgs::PointCloud2>("laser_detector", 1);
            pub = node.advertise<youbot_pick_and_place::ObjectLocation>("laser_detector", 1);
            sub = node.subscribe("scan", 1, &Detector::callback, this);

            while (ros::ok()) {
                ros::spinOnce();
                ros::Duration(0.5).sleep();
            }
        };

        ~Detector() {};

        void callback(const sensor_msgs::LaserScan::ConstPtr& scanInput) {
            sensor_msgs::PointCloud2 pointCloud2;
            projector.projectLaser(*scanInput, pointCloud2);

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(pointCloud2, cloud);

            youbot_pick_and_place::ObjectLocation msg;
            pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud.begin();
            double x = it->x;
            double y = it->y;
            //const double rangeX = 1;
            //const double rangeY = 0.2;

            //Find the closest object
            for(;it!=cloud.end(); it++) {
                if(it->y < RANGE_Y && it->y > -RANGE_Y) {
                    std::cout << it->x << "," << it->y << std::endl;
                    if(it->x < x) {
                        x = it->x;
                        y = it->y;
                    }
                }
            }
            //ROS_INFO("index: %d", i++);

            //Pub obj's location
            if (x < RANGE_X) {
                msg.x = x;
                msg.y = y;
                pub.publish(msg);
            }
        }

    private:
        laser_geometry::LaserProjection projector;
        ros::NodeHandle node;
        ros::Publisher pub;
        ros::Subscriber sub;
        
};

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "object_detector");
    Detector detector;
    //ros::spin();

    return 0;
}
