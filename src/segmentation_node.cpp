#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Convert ROS PointCloud2 -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    ROS_INFO("Input cloud: %lu points", pcl_cloud->points.size());

    // Voxel grid downsampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);  // 5mm leaf size - tune for your objects
    voxel_filter.filter(*voxel_filtered);

    ROS_INFO("After voxel filtering: %lu points", voxel_filtered->points.size());

    // Convert back to ROS PointCloud2 and publish
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*voxel_filtered, output_msg);
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
