#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *pcl_cloud);
	
	ROS_INFO("Converted cloud with %lu points", pcl_cloud->points.size());

	sensor_msgs::PointCloud2 output_msg;
	pcl::toROSMsg(*pcl_cloud, output_msg);
	output_msg.header = msg->header;
	
	pub.publish(output_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "segmentation_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloudCallBack);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/object_point_cloud", 1);
	
	ros::spin();
	return 0;
}
