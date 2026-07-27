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

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
	ros::WallTime t_start = ros::WallTime::now();
    // Convert ROS PointCloud2 -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *pcl_cloud);
	ros::WallTime t_convert = ros::WallTime::now();

    ROS_INFO("Input cloud: %lu points", pcl_cloud->points.size());

    // --- Passthrough filtering: crop to workspace region ---
    // Z axis (depth from camera) - keep only the range where your workspace/table sits
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(pcl_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.25, 1.0);  //meters 
    pass_z.filter(*z_filtered);

    // X axis (lateral) - keep only the reachable workspace width
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(z_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.3, 0.3);  //meters
    pass_x.filter(*x_filtered);

	ros::WallTime t_passthrough = ros::WallTime::now();
	ROS_INFO("After passthrough: %lu points", x_filtered->points.size());

	// --- Voxel grid downsampling ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(x_filtered);
    voxel_filter.setLeafSize(0.0025f, 0.0025f, 0.0025f);
    voxel_filter.filter(*voxel_filtered);

	ros::WallTime t_voxel = ros::WallTime::now();
    ROS_INFO("After voxel filtering: %lu points", voxel_filtered->points.size());

	// --- RANSAC plane segmentation: find the table ---
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.0075);  // how close a point must be to count as "on the plane"
    seg.setInputCloud(voxel_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        ROS_WARN("Could not estimate a planar model for the given cloud.");
        return;
    }

    // ROS_INFO("Plane found with %lu inlier points (table)", inliers->indices.size());

    // --- Extract the outliers (everything NOT the table = objects) ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(voxel_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);  // true = keep everything EXCEPT the plane inliers
    extract.filter(*objects_cloud);

	ros::WallTime t_ransac = ros::WallTime::now();
    ROS_INFO("After plane removal: %lu points remain (objects)", objects_cloud->points.size());

	// --- Euclidean cluster extraction ---
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(objects_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);   // 2cm - max gap between points in the same cluster
    ec.setMinClusterSize(1000);       // discard clusters smaller than this (likely noise)
    ec.setMaxClusterSize(25000);    // discard implausibly large clusters
    ec.setSearchMethod(tree);
    ec.setInputCloud(objects_cloud);
    ec.extract(cluster_indices);

	ros::WallTime t_cluster = ros::WallTime::now();
    ROS_INFO("Found %lu clusters", cluster_indices.size());

    // --- Build a colored output cloud: one distinct color per cluster ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);

    // A small palette of visually distinct colors, cycled if there are more clusters than colors
    std::vector<std::array<uint8_t, 3>> palette = {
        {230, 25, 75},   {60, 180, 75},   {255, 225, 25},  {0, 130, 200},
        {245, 130, 48},  {145, 30, 180},  {70, 240, 240},  {240, 50, 230},
        {210, 245, 60},  {250, 190, 212}
    };

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        const auto& color = palette[cluster_id % palette.size()];

        for (int idx : indices.indices) {
            pcl::PointXYZRGB point = objects_cloud->points[idx];
            point.r = color[0];
            point.g = color[1];
            point.b = color[2];
            colored_clusters->points.push_back(point);
        }

		// ROS_INFO("  Cluster %d: %lu points, color (%d,%d,%d)",
        //           cluster_id, indices.indices.size(), color[0], color[1], color[2]);

        cluster_id++;
    }

    colored_clusters->width = colored_clusters->points.size();
    colored_clusters->height = 1;
    colored_clusters->is_dense = true;


    // Convert back to ROS PointCloud2 and publish
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*colored_clusters, output_msg);
    output_msg.header = msg->header;
	ros::WallTime t_end = ros::WallTime::now();

	ROS_INFO("convert: %.1fms | passthrough: %.1fms | voxel: %.1fms | ransac: %.1fms | cluster: %.1fms | color: %.1fms | total: %.1fms",
        (t_convert - t_start).toSec() * 1000,
        (t_passthrough - t_convert).toSec() * 1000,
        (t_voxel - t_passthrough).toSec() * 1000,
        (t_ransac - t_voxel).toSec() * 1000,
        (t_cluster - t_ransac).toSec() * 1000,
        (t_end - t_cluster).toSec() * 1000,
        (t_end - t_start).toSec() * 1000);

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
