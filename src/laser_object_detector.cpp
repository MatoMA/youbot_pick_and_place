#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class Detector {
    public:
        Detector():node("~") {
            //TODO topic name
            pub = node.advertise<std_msgs::String>("pub", 1);
            sub = node.subscribe("topic", 1, &Detector::callback, this);

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

            //TODO find object
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
