#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
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

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

class SegmentationNode {
public:
    SegmentationNode(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/camera/depth/color/points", 1, &SegmentationNode::cloudCallback, this);
        pub_colored_ = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/colored_point_cloud", 1);
        pub_largest_ = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/object_point_cloud", 1);
		pub_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/segmentation/debug_point_cloud", 1);
		pub_table_normal_ = nh.advertise<geometry_msgs::Vector3Stamped>("/segmentation/table_normal", 1);
    }

private:
	// Publishers/subscribers
    ros::Subscriber sub_;
    ros::Publisher pub_colored_;
    ros::Publisher pub_largest_;
    ros::Publisher pub_debug_;
	ros::Publisher pub_table_normal_;
	// Color filter params
	bool use_color_filter_ = true;
    int stand_r_= 0;
	int stand_g_ = 92 ;
	int stand_b_ = 255;
    double color_threshold_ = 175.0;

	// --- Robot self-filtering: model the arm as a cylinder centered on the
	// ra_flange local x-axis, and drop any point that falls inside it before
	// clustering, so the robot itself is never mistaken for the object. ---
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_};

	const std::string kRobotFrame_ = "ra_flange";
	static constexpr double kRobotRadius_ = 0.1;       // m
	static constexpr double kRobotHeight_ = 0.47;  // m
	static constexpr double kTfTimeoutSec_ = 0.2;

    // --- Temporal smoothing state ---
    bool have_confirmed_ = false;
    Eigen::Vector4f confirmed_centroid_ = Eigen::Vector4f::Zero();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr confirmed_cluster_{new pcl::PointCloud<pcl::PointXYZRGB>};
    int mismatch_count_ = 0;
	int missing_count_ = 0;

    static constexpr float kCentroidMatchThreshold = 0.01f;  // [cm] - "same object" tolerance
    static constexpr int kConfirmFramesRequired = 4;         // consecutive mismatched frames before accepting a change
	static constexpr int kMissingFramesRequired = 10;  // consecutive empty frames before clearing held result

	// --- Robot self-filtering: drop points that fall inside the hand's
	// bounding cylinder (radius kRobotRadius_,height kRobotHeight_,
	// axis = local x-axis of ra_flange). 
	void filterRobotPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
	                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
	                        const std_msgs::Header& header) {
		geometry_msgs::TransformStamped tf_stamped;
		try {
			tf_stamped = tf_buffer_.lookupTransform(
				kRobotFrame_, header.frame_id, ros::Time(0), ros::Duration(kTfTimeoutSec_));
		} catch (const tf2::TransformException& ex) {
			ROS_WARN_THROTTLE(1.0, "filterRobotPoints: could not look up %s -> %s (%s); "
			                        "skipping robot filtering for this frame.",
			                        header.frame_id.c_str(), kRobotFrame_.c_str(), ex.what());
			*output = *input;
			return;
		}

		const Eigen::Affine3f cloud_to_flange = tf2::transformToEigen(tf_stamped).cast<float>();
		output->points.clear();
		output->points.reserve(input->points.size());

		for (const auto& point : input->points) {
			const Eigen::Vector3f p_flange = cloud_to_flange * Eigen::Vector3f(point.x, point.y, point.z);
			const float radial = std::sqrt(p_flange.y() * p_flange.y() + p_flange.z() * p_flange.z());
			const bool inside_robot = (radial <= kRobotRadius_) &&(p_flange.x() <= kRobotHeight_);
			if (!inside_robot) {output->points.push_back(point);}
		}

		output->width = output->points.size();
		output->height = 1;
		output->is_dense = input->is_dense;
	}

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
			//ROS_INFO("Point color: (%d, %d, %d), distance to target: %.2f", point.r, point.g, point.b, dist);

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

	void handleMissingDetection(const std_msgs::Header& header, const std::string& reason) {
		missing_count_++;
		ROS_WARN_THROTTLE(1.0, "%s (missing %d/%d)", reason.c_str(), missing_count_, kMissingFramesRequired);

		if (missing_count_ >= kMissingFramesRequired) {
			if (have_confirmed_) {
				ROS_WARN("Object missing for %d consecutive frames - publishing empty cloud.", missing_count_);
			}
			have_confirmed_ = false;
			confirmed_cluster_->points.clear();
			publishEmptyResult(header);
			return;  
		}

		publishHeldResult(header);
	}

	void publishEmptyResult(const std_msgs::Header& header) {
		pcl::PointCloud<pcl::PointXYZRGB> empty_cloud;
		empty_cloud.width = 0;
		empty_cloud.height = 1;
		empty_cloud.is_dense = true;

		sensor_msgs::PointCloud2 empty_msg;
		pcl::toROSMsg(empty_cloud, empty_msg);
		empty_msg.header = header; 
		pub_largest_.publish(empty_msg);
	}

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*pcl_cloud, *cloud_no_nan, nan_indices);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_robot(new pcl::PointCloud<pcl::PointXYZRGB>);
		filterRobotPoints(cloud_no_nan, cloud_no_robot, msg->header);

		// Pointer to hold whichever cloud should feed into the Z-pass filter
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_for_z_filter = cloud_no_robot;

		if (use_color_filter_) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
			filterByColor(cloud_no_robot, color_filtered);
			input_for_z_filter = color_filtered;
			sensor_msgs::PointCloud2 debug_msg;
			pcl::toROSMsg(*color_filtered, debug_msg);
			debug_msg.header = msg->header;
			pub_debug_.publish(debug_msg);
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
			handleMissingDetection(msg->header, "No plane found.");
			return;
		}

		// --- Publish the table normal, derived directly from the plane fit ---
		Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		normal.normalize();
		geometry_msgs::Vector3Stamped normal_msg;
		normal_msg.header = msg->header;
		normal_msg.vector.x = normal.x();
		normal_msg.vector.y = normal.y();
		normal_msg.vector.z = normal.z();
		pub_table_normal_.publish(normal_msg);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(voxel_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*objects_cloud);

        if (objects_cloud->points.empty()) {
			handleMissingDetection(msg->header, "No points remain after plane removal.");
			return;
		}

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(objects_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(150);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(objects_cloud);
        ec.extract(cluster_indices);

        if (cluster_indices.empty()) {
			handleMissingDetection(msg->header, "No clusters found this frame.");
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

		missing_count_ = 0;  // reset missing counter since we have a valid cluster this frame

        // --- Temporal smoothing decision ---
        if (!have_confirmed_) {
            acceptCandidate(candidate_cluster, candidate_centroid);
        } else {
            float dist = (candidate_centroid - confirmed_centroid_).head<3>().norm();

            if (dist < kCentroidMatchThreshold) {
                // Same object, normal update - reset mismatch counter
                acceptCandidate(candidate_cluster, candidate_centroid);
            } else {
                // Suspicious jump - could be noise or a real change
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