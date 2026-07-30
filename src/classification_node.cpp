#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

#include <pcl_package/PrimitiveEstimate.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <string>

enum class PrimitiveClass { UNKNOWN, SPHERE, CYLINDER, FLAT_BOX };

std::string toString(PrimitiveClass c) {
    switch (c) {
        case PrimitiveClass::SPHERE:   return "SPHERE";
        case PrimitiveClass::CYLINDER: return "CYLINDER";
        case PrimitiveClass::FLAT_BOX: return "FLAT_BOX";
        default:                       return "UNKNOWN";
    }
}

class PCAClassificationNode {
public:
    PCAClassificationNode(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/segmentation/object_point_cloud", 1,
                             &PCAClassificationNode::cloudCallback, this);
        sub_table_normal_ = nh.subscribe("/segmentation/table_normal", 1,
                                   &PCAClassificationNode::tableNormalCallback, this);
        pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/classification/pca_axes", 1);

        pub_primitive_marker_ = nh.advertise<visualization_msgs::Marker>("/perception/primitive_marker", 1);

        pub_estimate_ = nh.advertise<pcl_package::PrimitiveEstimate>("/classification/raw_primitive_estimate", 1);
    }

private:
    ros::Subscriber sub_;
    ros::Subscriber sub_table_normal_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_primitive_marker_;
    ros::Publisher pub_estimate_;

    Eigen::Vector3f table_normal_ = Eigen::Vector3f(0, -1, 0);  // fallback default until first message
    bool have_table_normal_ = false;

    // --- Classification thresholds ---
    double sphere_ratio_threshold_ = 0.65;
    double angle_threshold_deg_ = 20.0;

    // --- Unified temporal voting state and parameters     ---
    std::map<PrimitiveClass, double> class_scores_ = {
        {PrimitiveClass::SPHERE, 0.0},
        {PrimitiveClass::CYLINDER, 0.0},
        {PrimitiveClass::FLAT_BOX, 0.0}
    };
    PrimitiveClass confirmed_class_ = PrimitiveClass::UNKNOWN;

    static constexpr double kMaxClassScore = 30.0;     // score cap per class
    static constexpr double kConfirmThreshold = 30.0;  // bar to GAIN confirmation (high)
    static constexpr double kExpireThreshold = 10.0;   // bar to KEEP confirmation (low - hysteresis gap)      

    void tableNormalCallback(const geometry_msgs::Vector3StampedConstPtr& msg) {
        table_normal_ = Eigen::Vector3f(msg->vector.x, msg->vector.y, msg->vector.z).normalized();
        have_table_normal_ = true;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty()) {
            resetClassification();
            publishRawEstimate(msg->header, false, "UNKNOWN", Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
            return;
        }

        // --- PCA  ---
        Eigen::Vector4f centroid4f;
        pcl::compute3DCentroid(*cloud, centroid4f);
        Eigen::Vector3f centroid = centroid4f.head<3>();

        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, centroid4f, covariance);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
        Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();

        int order[3] = {2, 1, 0};  // descending order
        float lambda[3];
        Eigen::Vector3f axis[3];
        for (int i = 0; i < 3; ++i) {
            lambda[i] = eigenvalues(order[i]);
            axis[i] = eigenvectors.col(order[i]);
        }

        publishAxisMarkers(msg->header, centroid, axis, lambda);

        if (!have_table_normal_) {
            ROS_WARN_THROTTLE(2.0, "No table normal received yet from segmentation node - skipping classification.");
            return;
        }

        // --- Classification decision for THIS frame only ---
        PrimitiveClass raw_class = classify(lambda, axis[0]);
        updateTemporalPersistence(raw_class);  // update the temporal voting state

        // --- FITTING ---

        if (confirmed_class_ == PrimitiveClass::SPHERE) {
            Eigen::Vector3f sphere_center;
            float sphere_radius;

            if (fitSphere(cloud, sphere_center, sphere_radius)) {
                // ---  RViz visualization ---
                //publishSphereMarker(msg->header, sphere_center, sphere_radius);
                publishPrimitiveMarker(msg->header, MarkerShape::SPHERE, sphere_center, Eigen::Quaternionf::Identity(),
                        Eigen::Vector3f(2.0f * sphere_radius, 2.0f * sphere_radius, 2.0f * sphere_radius), 1.0f, 0.6f, 0.0f);
                // ---  Publish raw estimate ---
                publishRawEstimate(msg->header, true, "SPHERE", sphere_center, Eigen::Quaternionf::Identity(),
                         2.0f * sphere_radius);
            } else {
                publishRawEstimate(msg->header, false, "SPHERE", Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
                ROS_WARN("Sphere fit degenerate (invalid radius) - skipping publish this frame.");
            }
        } else if (confirmed_class_ == PrimitiveClass::CYLINDER) {
            Eigen::Vector3f cyl_center;
            float cyl_radius, cyl_height;

            if (fitCylinder(cloud, table_normal_, cyl_center, cyl_radius, cyl_height)) {
                // ---  RViz visualization ---
                //publishCylinderMarker(msg->header, cyl_center, table_normal_, 2.0f*cyl_radius, cyl_height);
                Eigen::Vector3f local_z(0.0f, 0.0f, 1.0f);
                Eigen::Quaternionf cyl_q = Eigen::Quaternionf::FromTwoVectors(local_z, table_normal_);
                publishPrimitiveMarker(msg->header, MarkerShape::CYLINDER, cyl_center, cyl_q,
                        Eigen::Vector3f(2.0f * cyl_radius, 2.0f * cyl_radius, cyl_height), 0.0f, 0.6f, 1.0f);
                // ---  Publish raw estimate ---
                publishRawEstimate(msg->header, true, "CYLINDER", cyl_center, Eigen::Quaternionf::Identity(),
                         2.0f * cyl_radius, cyl_height);
            } else {
                publishRawEstimate(msg->header, false, "CYLINDER", Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
                ROS_WARN("Cylinder fit degenerate (invalid radius) - skipping publish this frame.");
            }
        } else if (confirmed_class_ == PrimitiveClass::FLAT_BOX) {
            Eigen::Vector3f box_center;
            Eigen::Quaternionf box_orientation;
            float box_width, box_thickness;
            const float kAssumedDepth = 0.10f;  // 10cm assumed depth for the occluded axis (not very important for grasping)

            if (fitBox(cloud, table_normal_, axis[0], kAssumedDepth,
                    box_center, box_orientation, box_width, box_thickness)) {
                // ---  RViz visualization ---
                // publishBoxMarker(msg->header, box_center, box_orientation, box_width, box_thickness, kAssumedDepth);
                publishPrimitiveMarker(msg->header, MarkerShape::BOX, box_center, box_orientation,
                        Eigen::Vector3f(box_width, kAssumedDepth, box_thickness), 0.2f, 1.0f, 0.2f);
                // ---  Publish raw estimate ---
                publishRawEstimate(msg->header, true, "FLAT_BOX", box_center, box_orientation,
                         0.0f, 0.0f, box_width, box_thickness, kAssumedDepth);
            } else {
                publishRawEstimate(msg->header, false, "FLAT_BOX", Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
                ROS_WARN("Box fit degenerate - skipping publish this frame.");
            }
        } else{
            publishRawEstimate(msg->header, false, "UNKNOWN", Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity());
        }

    }

    PrimitiveClass classify(float lambda[3], const Eigen::Vector3f& largest_axis) {
        // --- Sphere check: two biggest eigenvalues are similar ---
        double ratio_21 = static_cast<double>(lambda[1]) / lambda[0];
        if (ratio_21 > sphere_ratio_threshold_) {
            return PrimitiveClass::SPHERE;
        }

        // --- Not a sphere: use largest-axis orientation vs. table normal ---
        float cos_angle = std::abs(largest_axis.dot(table_normal_));
        cos_angle = std::min(1.0f, std::max(-1.0f, cos_angle));  
        double angle_deg = std::acos(cos_angle) * 180.0 / M_PI;

        if (angle_deg <= angle_threshold_deg_) {
            // Largest axis nearly aligned with table normal -> perpendicular to table plane
            return PrimitiveClass::CYLINDER;
        } else if (angle_deg >= (90.0 - angle_threshold_deg_)) {
            // Largest axis nearly perpendicular to table normal -> parallel to table plane
            return PrimitiveClass::FLAT_BOX;
        }

        return PrimitiveClass::UNKNOWN;
    }

    void resetClassification() {
        for (auto& [cls, score] : class_scores_) score = 0.0;
        confirmed_class_ = PrimitiveClass::UNKNOWN;
    }

    // --- Combined entry point, called once per frame with this frame's raw classification ---
    void updateTemporalPersistence(PrimitiveClass raw_class) {
        updateClassScores(raw_class);
        updateConfirmedClass();
    }

    // --- Step 1: update the bounded score of every candidate class from this frame's raw vote ---
    void updateClassScores(PrimitiveClass raw_class) {
        for (auto& [cls, score] : class_scores_) {
            if (cls == raw_class) {
                score = std::min(kMaxClassScore, score + 1.0);
            //} else if (raw_class != PrimitiveClass::UNKNOWN) {
            } else {  // UNKNOWN votes against all classes, so decrement all scores
                score = std::max(0.0, score - 1.0);
            }
        }
    }

    // --- Step 2: apply hysteresis to decide whether confirmed_class_ should change ---
    void updateConfirmedClass() {
        // Find the current leading candidate
        PrimitiveClass leading_class = PrimitiveClass::UNKNOWN;
        double leading_score = 0.0;
        for (const auto& [cls, score] : class_scores_) {
            if (score > leading_score) {
                leading_score = score;
                leading_class = cls;
            }
        }

        // Gain: a different class convincingly crosses the high bar
        if (leading_score >= kConfirmThreshold && leading_class != confirmed_class_) {
            ROS_INFO("Classification CONFIRMED: %s (score %.1f)",
                      toString(leading_class).c_str(), leading_score);
            confirmed_class_ = leading_class;
            return;
        }

        // Keep/expire: only the currently confirmed class's own score matters here
        if (confirmed_class_ != PrimitiveClass::UNKNOWN) {
            double current_score = class_scores_[confirmed_class_];
            if (current_score < kExpireThreshold) {
                ROS_WARN("Confirmed class %s lost support (score=%.1f) - clearing.",
                          toString(confirmed_class_).c_str(), current_score);
                confirmed_class_ = PrimitiveClass::UNKNOWN;
            }
        }
    }

    // Algebraic least-squares sphere fit.
    // Sphere equation: (x-cx)^2 + (y-cy)^2 + (z-cz)^2 = r^2
    // Rearranged (linear in the unknowns cx,cy,cz,k):
    //   x^2+y^2+z^2 = 2*cx*x + 2*cy*y + 2*cz*z + k,   where k = r^2 - cx^2 - cy^2 - cz^2
    // Solving this linear system directly gives center + radius, no iteration needed.
    bool fitSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                    Eigen::Vector3f& center_out, float& radius_out) {
        int n = cloud->points.size();
        Eigen::MatrixXf A(n, 4);
        Eigen::VectorXf b(n);

        for (int i = 0; i < n; ++i) {
            const auto& p = cloud->points[i];
            A(i, 0) = 2.0f * p.x;
            A(i, 1) = 2.0f * p.y;
            A(i, 2) = 2.0f * p.z;
            A(i, 3) = 1.0f;
            b(i) = p.x * p.x + p.y * p.y + p.z * p.z;
        }

        Eigen::Vector4f sol = A.colPivHouseholderQr().solve(b);
        center_out = sol.head<3>();
        float k = sol(3);
        float r_squared = k + center_out.squaredNorm();

        if (r_squared <= 0.0f) {
            return false;  // degenerate fit - reject
        }
        radius_out = std::sqrt(r_squared);
        return true;
    }

    bool fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                    const Eigen::Vector3f& axis,
                    Eigen::Vector3f& center_out, float& radius_out, float& height_out) {
        Eigen::Vector4f centroid4f;
        pcl::compute3DCentroid(*cloud, centroid4f);
        Eigen::Vector3f centroid = centroid4f.head<3>();

        Eigen::Vector3f ref = (std::abs(axis.x()) < 0.9f) ? Eigen::Vector3f(1, 0, 0) : Eigen::Vector3f(0, 1, 0);
        Eigen::Vector3f u = axis.cross(ref).normalized();
        Eigen::Vector3f v = axis.cross(u).normalized();

        int n = cloud->points.size();
        Eigen::MatrixXf A(n, 3);
        Eigen::VectorXf b(n);

        // Only need xp, yp here for the radius fit - axis projections are now
        // handled separately by robustBoundsAlongAxis below.
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3f rel(cloud->points[i].x - centroid.x(),
                                cloud->points[i].y - centroid.y(),
                                cloud->points[i].z - centroid.z());

            float xp = rel.dot(u);
            float yp = rel.dot(v);
            A(i, 0) = 2.0f * xp;
            A(i, 1) = 2.0f * yp;
            A(i, 2) = 1.0f;
            b(i) = xp * xp + yp * yp;
        }

        Eigen::Vector3f sol = A.colPivHouseholderQr().solve(b);
        float a_local = sol(0), b_local = sol(1), k = sol(2);
        float r_squared = k + a_local * a_local + b_local * b_local;

        if (r_squared <= 0.0f) {
            return false;
        }

        radius_out = std::sqrt(r_squared);

        // Reuse the shared helper instead of a hand-rolled loop + robustBounds
        auto [low, high] = robustBoundsAlongAxis(cloud, centroid, axis, 0.02, 0.98);
        height_out = high - low;
        float mid_axis = (low + high) / 2.0f;

        center_out = centroid + a_local * u + b_local * v + mid_axis * axis;

        return true;
    }

    bool fitBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
            const Eigen::Vector3f& normal, const Eigen::Vector3f& largest_axis,
            float assumed_depth,
            Eigen::Vector3f& center_out, Eigen::Quaternionf& orientation_out,
            float& width_out, float& thickness_out) {
        Eigen::Vector4f centroid4f;
        pcl::compute3DCentroid(*cloud, centroid4f);
        Eigen::Vector3f centroid = centroid4f.head<3>();

        // Width direction: largest PCA axis, orthogonalized against the table normal
        Eigen::Vector3f width_dir = (largest_axis - largest_axis.dot(normal) * normal).normalized();
        Eigen::Vector3f depth_dir = normal.cross(width_dir).normalized();

        // Orient depth_dir so that POSITIVE = away from camera.
        if (depth_dir.dot(centroid) < 0.0f) {
            depth_dir = -depth_dir;
        }

        auto [thick_low, thick_high] = robustBoundsAlongAxis(cloud, centroid, normal, 0.02, 0.98);
        thickness_out = thick_high - thick_low;

        auto [width_low, width_high] = robustBoundsAlongAxis(cloud, centroid, width_dir, 0.02, 0.98);
        width_out = width_high - width_low;

        if (thickness_out <= 0.0f || width_out <= 0.0f) {
            return false;
        }

        auto [depth_low, depth_high] = robustBoundsAlongAxis(cloud, centroid, depth_dir, 0.02, 0.98);

        // --- Depth is assumed since it very likely that this axis is occluded and is not very important for the grasping task.
        float center_offset_along_depth = depth_low + assumed_depth / 2.0f;
        center_out = centroid + center_offset_along_depth * depth_dir;

        Eigen::Matrix3f rot;
        rot.col(0) = width_dir;
        rot.col(1) = depth_dir;
        rot.col(2) = normal;
        orientation_out = Eigen::Quaternionf(rot);

        return true;
    }

    // Robust (percentile-based) bounds of a point cloud's projection onto an arbitrary axis,
    // relative to a given reference point. Returns {low, high} along that axis.
    std::pair<float, float> robustBoundsAlongAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                                    const Eigen::Vector3f& reference,
                                                    const Eigen::Vector3f& axis,
                                                    double percentile_low, double percentile_high) {
        std::vector<float> projections;
        projections.reserve(cloud->points.size());

        for (const auto& p : cloud->points) {
            Eigen::Vector3f rel(p.x - reference.x(), p.y - reference.y(), p.z - reference.z());
            projections.push_back(rel.dot(axis));
        }

        std::sort(projections.begin(), projections.end());
        int n = projections.size();
        int idx_low = static_cast<int>(percentile_low * (n - 1));
        int idx_high = static_cast<int>(percentile_high * (n - 1));

        return {projections[idx_low], projections[idx_high]};
    }

    void publishRawEstimate(const std_msgs::Header& header, bool valid, const std::string& type,
                          const Eigen::Vector3f& position, const Eigen::Quaternionf& orientation,
                          float diameter = 0.0f, float height = 0.0f,
                          float width = 0.0f, float thickness = 0.0f, float depth = 0.0f) {
        pcl_package::PrimitiveEstimate msg;
        msg.primitive_type = type;
        msg.valid = valid;

        msg.pose.header = header;
        msg.pose.pose.position.x = position.x();
        msg.pose.pose.position.y = position.y();
        msg.pose.pose.position.z = position.z();
        msg.pose.pose.orientation.x = orientation.x();
        msg.pose.pose.orientation.y = orientation.y();
        msg.pose.pose.orientation.z = orientation.z();
        msg.pose.pose.orientation.w = orientation.w();

        msg.diameter = diameter;
        msg.height = height;
        msg.width = width;
        msg.thickness = thickness;
        msg.depth = depth;

        pub_estimate_.publish(msg);
    }


    enum class MarkerShape { SPHERE, CYLINDER, BOX };

    // Visualization marker publisher for all three primitive types.
    void publishPrimitiveMarker(const std_msgs::Header& header, MarkerShape shape,
                                const Eigen::Vector3f& center, const Eigen::Quaternionf& orientation,
                                const Eigen::Vector3f& scale,
                                float r, float g, float b) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "fitted_primitive";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;

        switch (shape) {
            case MarkerShape::SPHERE:   marker.type = visualization_msgs::Marker::SPHERE;   break;
            case MarkerShape::CYLINDER: marker.type = visualization_msgs::Marker::CYLINDER; break;
            case MarkerShape::BOX:      marker.type = visualization_msgs::Marker::CUBE;     break;
        }

        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        marker.pose.orientation.x = orientation.x();
        marker.pose.orientation.y = orientation.y();
        marker.pose.orientation.z = orientation.z();
        marker.pose.orientation.w = orientation.w();

        marker.scale.x = scale.x();
        marker.scale.y = scale.y();
        marker.scale.z = scale.z();

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration(0.5);
        pub_primitive_marker_.publish(marker);
    }
    // Maybe all markers could be published on a single topic? would make it cleaner to manage in RViz
    // void publishSphereMarker(const std_msgs::Header& header, const Eigen::Vector3f& center, float radius) {
    //     visualization_msgs::Marker marker;
    //     marker.header = header;
    //     marker.ns = "fitted_sphere";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.pose.position.x = center.x();
    //     marker.pose.position.y = center.y();
    //     marker.pose.position.z = center.z();
    //     marker.pose.orientation.x = 0.0;
    //     marker.pose.orientation.y = 0.0;
    //     marker.pose.orientation.z = 0.0;
    //     marker.pose.orientation.w = 1.0;  // orientation irrelevant for a sphere

    //     marker.scale.x = 2.0 * radius;
    //     marker.scale.y = 2.0 * radius;
    //     marker.scale.z = 2.0 * radius;

    //     marker.color.r = 1.0f;
    //     marker.color.g = 0.6f;
    //     marker.color.b = 0.0f;
    //     marker.color.a = 1.0f;  

    //     marker.lifetime = ros::Duration(0.5);  
    //     pub_sphere_marker_.publish(marker);
    // }

    // void publishCylinderMarker(const std_msgs::Header& header, const Eigen::Vector3f& center,
    //                          const Eigen::Vector3f& axis, float radius, float height) {
    //     visualization_msgs::Marker marker;
    //     marker.header = header;
    //     marker.ns = "fitted_cylinder";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::CYLINDER;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.pose.position.x = center.x();
    //     marker.pose.position.y = center.y();
    //     marker.pose.position.z = center.z();

    //     // RViz's CYLINDER marker is aligned with the LOCAL Z axis by default,
    //     // so we need a rotation that maps local Z onto our fitted axis direction.
    //     Eigen::Vector3f local_z(0.0f, 0.0f, 1.0f);
    //     Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(local_z, axis);

    //     marker.pose.orientation.x = q.x();
    //     marker.pose.orientation.y = q.y();
    //     marker.pose.orientation.z = q.z();
    //     marker.pose.orientation.w = q.w();

    //     marker.scale.x = 2.0 * radius;  
    //     marker.scale.y = 2.0 * radius;  
    //     marker.scale.z = height;        

    //     marker.color.r = 0.0f;
    //     marker.color.g = 0.6f;
    //     marker.color.b = 1.0f;
    //     marker.color.a = 1.0f;

    //     marker.lifetime = ros::Duration(0.5);
    //     pub_cylinder_marker_.publish(marker);
    // }

    // void publishBoxMarker(const std_msgs::Header& header, const Eigen::Vector3f& center,
    //                    const Eigen::Quaternionf& orientation, float width, float thickness, float depth) {
    //     visualization_msgs::Marker marker;
    //     marker.header = header;
    //     marker.ns = "fitted_box";
    //     marker.id = 0;
    //     marker.type = visualization_msgs::Marker::CUBE;
    //     marker.action = visualization_msgs::Marker::ADD;

    //     marker.pose.position.x = center.x();
    //     marker.pose.position.y = center.y();
    //     marker.pose.position.z = center.z();
    //     marker.pose.orientation.x = orientation.x();
    //     marker.pose.orientation.y = orientation.y();
    //     marker.pose.orientation.z = orientation.z();
    //     marker.pose.orientation.w = orientation.w();

    //     marker.scale.x = width;
    //     marker.scale.y = depth;
    //     marker.scale.z = thickness;

    //     marker.color.r = 0.2f;
    //     marker.color.g = 1.0f;
    //     marker.color.b = 0.2f;
    //     marker.color.a = 1.0f;

    //     marker.lifetime = ros::Duration(0.5);
    //     pub_box_marker_.publish(marker);
    // }

    void publishAxisMarkers(const std_msgs::Header& header, const Eigen::Vector3f& centroid,
                             Eigen::Vector3f axis[3], float lambda[3]) {
        visualization_msgs::MarkerArray marker_array;
        const float viz_scale = 3.0f;
        std::array<std::array<float, 3>, 3> colors = {{
            {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}
        }};

        for (int i = 0; i < 3; ++i) {
            float length = std::sqrt(std::max(lambda[i], 0.0f)) * viz_scale;

            visualization_msgs::Marker arrow;
            arrow.header = header;
            arrow.ns = "pca_axes";
            arrow.id = i;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.pose.orientation.w = 1.0;

            // Explicitly set identity orientation to avoid the "uninitialized quaternion" warning
            arrow.pose.orientation.x = 0.0;
            arrow.pose.orientation.y = 0.0;
            arrow.pose.orientation.z = 0.0;
            arrow.pose.orientation.w = 1.0;

            geometry_msgs::Point start, end;
            start.x = centroid.x(); start.y = centroid.y(); start.z = centroid.z();
            end.x = centroid.x() + axis[i].x() * length;
            end.y = centroid.y() + axis[i].y() * length;
            end.z = centroid.z() + axis[i].z() * length;
            arrow.points.push_back(start);
            arrow.points.push_back(end);

            arrow.scale.x = 0.005; arrow.scale.y = 0.01; arrow.scale.z = 0.01;
            arrow.color.r = colors[i][0]; arrow.color.g = colors[i][1]; arrow.color.b = colors[i][2];
            arrow.color.a = 1.0;
            arrow.lifetime = ros::Duration(0.5);

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