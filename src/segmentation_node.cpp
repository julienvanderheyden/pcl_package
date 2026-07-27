#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

class SegmentationNode {
public:
    SegmentationNode(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/camera/depth/color/points", 1, &SegmentationNode::cloudCallback, this);
        pub_colored_ = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/colored_point_cloud", 1);
        pub_largest_ = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/object_point_cloud", 1);
    }

private:
	// Publishers/subscribers
    ros::Subscriber sub_;
    ros::Publisher pub_colored_;
    ros::Publisher pub_largest_;

	// Color filter params
	bool use_color_filter_ = true;
    int stand_r_= 0;
	int stand_g_ = 92 ;
	int stand_b_ = 255;
    double color_threshold_ = 60.0;

    // --- Temporal smoothing state ---
    bool have_confirmed_ = false;
    Eigen::Vector4f confirmed_centroid_ = Eigen::Vector4f::Zero();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr confirmed_cluster_{new pcl::PointCloud<pcl::PointXYZRGB>};
    int mismatch_count_ = 0;

    static constexpr float kCentroidMatchThreshold = 0.01f;  // [cm] - "same object" tolerance
    static constexpr int kConfirmFramesRequired = 4;         // consecutive mismatched frames before accepting a change

	// --- Color filtering: remove points whose color is close to a given target ---
    void filterByColor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) {
        output->points.clear();
        output->points.reserve(input->points.size());

        for (const auto& point : input->points) {
            float dr = static_cast<float>(point.r) - stand_r_;
            float dg = static_cast<float>(point.g) - stand_g_;
            float db = static_cast<float>(point.b) - stand_b_;
            double dist = std::sqrt(dr * dr + dg * dg + db * db);

            if (dist >= color_threshold_) {
                // Color is far enough from the target - keep this point
                output->points.push_back(point);
            }
            // else: too close to target color (likely the stand) - drop it
        }

        output->width = output->points.size();
        output->height = 1;
        output->is_dense = input->is_dense;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*pcl_cloud, *cloud_no_nan, nan_indices);

		// Pointer to hold whichever cloud should feed into the Z-pass filter
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_for_z_filter = cloud_no_nan;

		if (use_color_filter_) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
			filterByColor(cloud_no_nan, color_filtered);
			input_for_z_filter = color_filtered;
		}

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(input_for_z_filter);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(0.25, 1.0);
        pass_z.filter(*z_filtered);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud(z_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-0.4, 0.4);
        pass_x.filter(*x_filtered);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(x_filtered);
        voxel_filter.setLeafSize(0.0025f, 0.0025f, 0.0025f);
        voxel_filter.filter(*voxel_filtered);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(voxel_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            ROS_WARN("No plane found.");
            publishHeldResult(msg->header);
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(voxel_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*objects_cloud);

        if (objects_cloud->points.empty()) {
            ROS_WARN("No points remain after plane removal.");
            publishHeldResult(msg->header);
            return;
        }

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(objects_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(1000);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(objects_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
            ROS_WARN("No clusters found this frame.");
            publishHeldResult(msg->header);
            return;
        }

		// --- Build the colored multi-cluster cloud (debug visualization, always published from raw frame) ---
		static const std::vector<std::array<uint8_t, 3>> palette = {
			{230, 25, 75},   {60, 180, 75},   {255, 225, 25},  {0, 130, 200},
			{245, 130, 48},  {145, 30, 180},  {70, 240, 240},  {240, 50, 230}
		};

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
		size_t total_clustered_points = 0;
		for (const auto& indices : cluster_indices) total_clustered_points += indices.indices.size();
		colored_clusters->points.reserve(total_clustered_points);

		// --- Find largest cluster in the SAME pass as coloring ---
		size_t largest_idx = 0, largest_size = 0;

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
			if (indices.indices.size() > largest_size) {
				largest_size = indices.indices.size();
				largest_idx = cluster_id;
			}
			cluster_id++;
		}
		colored_clusters->width = colored_clusters->points.size();
		colored_clusters->height = 1;
		colored_clusters->is_dense = true;

		sensor_msgs::PointCloud2 colored_msg;
		pcl::toROSMsg(*colored_clusters, colored_msg);
		colored_msg.header = msg->header;
		pub_colored_.publish(colored_msg);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr candidate_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        candidate_cluster->points.reserve(largest_size);
        for (int idx : cluster_indices[largest_idx].indices) {
            candidate_cluster->points.push_back(objects_cloud->points[idx]);
        }
        candidate_cluster->width = candidate_cluster->points.size();
        candidate_cluster->height = 1;
        candidate_cluster->is_dense = true;

        Eigen::Vector4f candidate_centroid;
        pcl::compute3DCentroid(*candidate_cluster, candidate_centroid);

        // --- Temporal smoothing decision ---
        if (!have_confirmed_) {
            // First valid frame ever - accept immediately
            acceptCandidate(candidate_cluster, candidate_centroid);
        } else {
            float dist = (candidate_centroid - confirmed_centroid_).head<3>().norm();

            if (dist < kCentroidMatchThreshold) {
                // Same object, normal update - reset mismatch counter
                acceptCandidate(candidate_cluster, candidate_centroid);
            } else {
                // Suspicious jump - could be noise (split/fusion) or a real change
                mismatch_count_++;

                if (mismatch_count_ >= kConfirmFramesRequired) {
                    // Persisted long enough - accept it as a real change
                    acceptCandidate(candidate_cluster, candidate_centroid);
                }
            }
        }

        publishHeldResult(msg->header);
    }

    void acceptCandidate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster, const Eigen::Vector4f& centroid) {
        confirmed_cluster_ = cluster;
        confirmed_centroid_ = centroid;
        have_confirmed_ = true;
        mismatch_count_ = 0;
    }

    void publishHeldResult(const std_msgs::Header& header) {
        if (!have_confirmed_) return;  // nothing valid yet

        sensor_msgs::PointCloud2 largest_msg;
        pcl::toROSMsg(*confirmed_cluster_, largest_msg);
        largest_msg.header = header;
        pub_largest_.publish(largest_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "segmentation_node");
    ros::NodeHandle nh;
    SegmentationNode node(nh);
    ros::spin();
    return 0;
}