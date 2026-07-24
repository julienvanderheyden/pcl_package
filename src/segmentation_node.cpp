#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // Convert ROS PointCloud2 -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    ROS_INFO("Input cloud: %lu points", pcl_cloud->points.size());

    // --- Voxel grid downsampling ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.0025f, 0.0025f, 0.0025f);
    voxel_filter.filter(*voxel_filtered);

    ROS_INFO("After voxel filtering: %lu points", voxel_filtered->points.size());

    // --- Passthrough filtering: crop to workspace region ---
    // Z axis (depth from camera) - keep only the range where your workspace/table sits
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(voxel_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.3, 1.0);  //meters 
    pass_z.filter(*z_filtered);

    // X axis (lateral) - keep only the reachable workspace width
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(z_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.3, 0.3);  //meters
    pass_x.filter(*x_filtered);

	ROS_INFO("After passthrough: %lu points", x_filtered->points.size());

	// --- RANSAC plane segmentation: find the table ---
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.0075);  // how close a point must be to count as "on the plane"
    seg.setInputCloud(x_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("Could not estimate a planar model for the given cloud.");
        return;
    }

    // ROS_INFO("Plane found with %lu inlier points (table)", inliers->indices.size());

    // --- Extract the outliers (everything NOT the table = objects) ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(x_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);  // true = keep everything EXCEPT the plane inliers
    extract.filter(*objects_cloud);

    ROS_INFO("After plane removal: %lu points remain (objects)", objects_cloud->points.size());

    // Convert back to ROS PointCloud2 and publish
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*objects_cloud, output_msg);
    output_msg.header = msg->header;

    pub.publish(output_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "segmentation_node");
	ros::NodeHandle nh;
	
	ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, cloudCallback);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/object_point_cloud", 1);
	
	ros::spin();
	return 0;
}
