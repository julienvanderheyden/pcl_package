#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>

class PCAClassificationNode {
public:
    PCAClassificationNode(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/segmentation/object_point_cloud", 1,
                             &PCAClassificationNode::cloudCallback, this);
        pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/classification/pca_axes", 1);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_markers_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.size() < 10) {
            ROS_WARN("Too few points (%lu) for reliable PCA, skipping.", cloud->points.size());
            return;
        }

        // --- Centroid ---
        Eigen::Vector4f centroid4f;
        pcl::compute3DCentroid(*cloud, centroid4f);
        Eigen::Vector3f centroid = centroid4f.head<3>();

        // --- Covariance matrix (about the centroid) ---
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid4f, covariance);

        // --- Eigen decomposition ---
        // SelfAdjointEigenSolver is appropriate since covariance matrices are symmetric
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);

        // Eigen returns eigenvalues in ASCENDING order - we want descending (largest spread first)
        Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();

        // Reorder: index 0 = largest eigenvalue (lambda1), index 2 = smallest (lambda3)
        int order[3] = {2, 1, 0};
        float lambda[3];
        Eigen::Vector3f axis[3];
        for (int i = 0; i < 3; ++i) {
            lambda[i] = eigenvalues(order[i]);
            axis[i] = eigenvectors.col(order[i]);
        }

        // --- Log eigenvalues and ratios ---
        ROS_INFO("---");
        ROS_INFO("Points: %lu", cloud->points.size());
        ROS_INFO("Eigenvalues: L1=%.6f  L2=%.6f  L3=%.6f", lambda[0], lambda[1], lambda[2]);
        ROS_INFO("Ratios: L2/L1=%.3f  L3/L1=%.3f  L3/L2=%.3f",
                  lambda[1] / lambda[0], lambda[2] / lambda[0], lambda[2] / lambda[1]);
        ROS_INFO("Axis1 (largest spread): [%.3f, %.3f, %.3f]", axis[0].x(), axis[0].y(), axis[0].z());
        ROS_INFO("Axis2:                  [%.3f, %.3f, %.3f]", axis[1].x(), axis[1].y(), axis[1].z());
        ROS_INFO("Axis3 (smallest spread):[%.3f, %.3f, %.3f]", axis[2].x(), axis[2].y(), axis[2].z());

        // --- Publish markers for visual inspection in RViz ---
        publishAxisMarkers(msg->header, centroid, axis, lambda);
    }

    void publishAxisMarkers(const std_msgs::Header& header, const Eigen::Vector3f& centroid,
                             Eigen::Vector3f axis[3], float lambda[3]) {
        visualization_msgs::MarkerArray marker_array;

        // Scale arrow length by sqrt(eigenvalue) - proportional to std. deviation along that axis
        // Multiplied by a visualization factor so arrows are visible at typical object scales
        const float viz_scale = 3.0f;
        std::array<std::array<float, 3>, 3> colors = {{
            {1.0f, 0.0f, 0.0f},  // axis 1: red
            {0.0f, 1.0f, 0.0f},  // axis 2: green
            {0.0f, 0.0f, 1.0f}   // axis 3: blue
        }};

        for (int i = 0; i < 3; ++i) {
            float length = std::sqrt(std::max(lambda[i], 0.0f)) * viz_scale;

            visualization_msgs::Marker arrow;
            arrow.header = header;
            arrow.ns = "pca_axes";
            arrow.id = i;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start, end;
            start.x = centroid.x();
            start.y = centroid.y();
            start.z = centroid.z();
            end.x = centroid.x() + axis[i].x() * length;
            end.y = centroid.y() + axis[i].y() * length;
            end.z = centroid.z() + axis[i].z() * length;

            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.005;  // shaft diameter
            arrow.scale.y = 0.01;   // head diameter
            arrow.scale.z = 0.01;   // head length

            arrow.color.r = colors[i][0];
            arrow.color.g = colors[i][1];
            arrow.color.b = colors[i][2];
            arrow.color.a = 1.0;

            arrow.lifetime = ros::Duration(0.5);  // auto-expire if node stops publishing

            marker_array.markers.push_back(arrow);
        }

        pub_markers_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pca_classification_node");
    ros::NodeHandle nh;
    PCAClassificationNode node(nh);
    ros::spin();
    return 0;
}