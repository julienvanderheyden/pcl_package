#include <ros/ros.h>
#include <pcl_package/PrimitiveEstimate.h>
#include <pcl_package/GetStableEstimate.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <deque>
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

// PrimitiveEstimate has no top-level `header` field (only pose.header), so the
// default TimeStamp<M> trait (which reads m.header.stamp) won't compile/sync.
// This specialization points message_filters at the nested stamp instead.
// classification_node.cpp sets msg.pose.header from the same
// /segmentation/object_point_cloud message this node also subscribes to, so
// the timestamps already line up - this just exposes that to the sync policy.
namespace ros {
namespace message_traits {

template <>
struct TimeStamp<pcl_package::PrimitiveEstimate> {
    static ros::Time value(const pcl_package::PrimitiveEstimate& m) {
        return m.pose.header.stamp;
    }
};

}  // namespace message_traits
}  // namespace ros

class PerceptionNode {
public:
    PerceptionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        pnh.param("consensus_leaf_size", leaf_size_, 0.005);   // reuse segmentation node's voxel leaf size if possible
        pnh.param("consensus_ratio", consensus_ratio_, 0.4);   // fraction of frames a voxel must appear in to survive
        pnh.param("sync_slop", sync_slop_, 0.05);               // seconds, approximate-time sync tolerance

        estimate_sub_.subscribe(nh, "/classification/raw_primitive_estimate", 1);
        cloud_sub_.subscribe(nh, "/segmentation/object_point_cloud", 1);

        sync_.reset(new Synchronizer(SyncPolicy(10), estimate_sub_, cloud_sub_));
        sync_->setMaxIntervalDuration(ros::Duration(sync_slop_));
        sync_->registerCallback(boost::bind(&PerceptionNode::syncedCallback, this, _1, _2));

        service_ = nh.advertiseService("/perception/get_stable_estimate",
                                         &PerceptionNode::handleRequest, this);

        pub_final_marker_ = nh.advertise<visualization_msgs::Marker>("/perception/final_primitive_marker", 1, /*latch=*/true);
        pub_debug_consensus_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/perception/debug_consensus_cloud", 1);
    }

private:
    message_filters::Subscriber<pcl_package::PrimitiveEstimate> estimate_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<
        pcl_package::PrimitiveEstimate, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    ros::ServiceServer service_;
    ros::Publisher pub_final_marker_;
    ros::Publisher pub_debug_consensus_cloud_;

    static constexpr size_t kWindowSize = 20;
    static constexpr size_t kMinSamplesRequired = 10;  // half the window

    double leaf_size_;
    double consensus_ratio_;
    double sync_slop_;

    std::string current_type_ = "UNKNOWN";
    std::deque<pcl_package::PrimitiveEstimate> buffer_;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer_;  // kept in lockstep with buffer_

    void syncedCallback(const pcl_package::PrimitiveEstimateConstPtr& msg,
                         const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        if (!msg->valid) {
            // No object / no confirmed classification this frame - clear everything
            buffer_.clear();
            cloud_buffer_.clear();
            current_type_ = "UNKNOWN";
            return;
        }

        if (msg->primitive_type != current_type_) {
            // Classification changed - old samples no longer apply
            buffer_.clear();
            cloud_buffer_.clear();
            current_type_ = msg->primitive_type;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *cloud);

        buffer_.push_back(*msg);
        cloud_buffer_.push_back(cloud);

        if (buffer_.size() > kWindowSize) {
            buffer_.pop_front();
            cloud_buffer_.pop_front();
        }

        // Debug visualization: publish the consensus cloud computed over whatever
        // is currently buffered, regardless of kMinSamplesRequired. With very few
        // frames buffered every voxel trivially meets consensus_ratio_, so early
        // on this will look like the raw cloud - that's expected, not a bug.
        if (pub_debug_consensus_cloud_.getNumSubscribers() > 0) {
            pub_debug_consensus_cloud_.publish(computeConsensusCloud(cloud_msg->header));
        }
    }

    float median(std::vector<float> values) const {
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        return (n % 2 == 0) ? (values[n/2 - 1] + values[n/2]) / 2.0f : values[n/2];
    }

    // Voxel-consensus outlier filter: keeps only points whose voxel is observed
    // in at least `consensus_ratio_` of the buffered frames, and replaces each
    // surviving voxel with the centroid of the points that landed in it.
    // This rejects transient noise/outliers (present in only a frame or two)
    // without accumulating density the way a plain concatenate+downsample would.
    sensor_msgs::PointCloud2 computeConsensusCloud(const std_msgs::Header& header) const {
        struct VoxelKey {
            int x, y, z;
            bool operator==(const VoxelKey& o) const {
                return x == o.x && y == o.y && z == o.z;
            }
        };
        struct VoxelKeyHash {
            size_t operator()(const VoxelKey& k) const {
                return (static_cast<size_t>(k.x) * 73856093) ^
                       (static_cast<size_t>(k.y) * 19349663) ^
                       (static_cast<size_t>(k.z) * 83492791);
            }
        };
        struct VoxelAccum {
            double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
            int point_count = 0;
            std::set<int> frames_seen;
        };

        std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxels;

        for (size_t frame_idx = 0; frame_idx < cloud_buffer_.size(); ++frame_idx) {
            const auto& cloud = cloud_buffer_[frame_idx];
            for (const auto& pt : cloud->points) {
                if (!pcl::isFinite(pt)) continue;

                VoxelKey key{
                    static_cast<int>(std::floor(pt.x / leaf_size_)),
                    static_cast<int>(std::floor(pt.y / leaf_size_)),
                    static_cast<int>(std::floor(pt.z / leaf_size_))
                };

                auto& v = voxels[key];
                v.sum_x += pt.x;
                v.sum_y += pt.y;
                v.sum_z += pt.z;
                v.point_count++;
                v.frames_seen.insert(static_cast<int>(frame_idx));
            }
        }

        size_t min_frames = std::max<size_t>(
            1, static_cast<size_t>(std::ceil(consensus_ratio_ * cloud_buffer_.size())));

        pcl::PointCloud<pcl::PointXYZ> consensus_cloud;
        consensus_cloud.reserve(voxels.size());
        for (const auto& kv : voxels) {
            const auto& v = kv.second;
            if (v.frames_seen.size() >= min_frames) {
                consensus_cloud.points.emplace_back(
                    static_cast<float>(v.sum_x / v.point_count),
                    static_cast<float>(v.sum_y / v.point_count),
                    static_cast<float>(v.sum_z / v.point_count));
            }
        }
        consensus_cloud.width = consensus_cloud.points.size();
        consensus_cloud.height = 1;
        consensus_cloud.is_dense = true;

        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(consensus_cloud, out);
        out.header = header;
        return out;
    }

    void publishFinalMarker(const pcl_package::PrimitiveEstimate& estimate) {
        visualization_msgs::Marker marker;
        marker.header = estimate.pose.header;
        marker.ns = "final_primitive";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose = estimate.pose.pose;

        if (estimate.primitive_type == "SPHERE") {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = estimate.diameter;
            marker.scale.y = estimate.diameter;
            marker.scale.z = estimate.diameter;
            marker.color.r = 1.0f; marker.color.g = 0.6f; marker.color.b = 0.0f;
        } else if (estimate.primitive_type == "CYLINDER") {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.scale.x = estimate.diameter;
            marker.scale.y = estimate.diameter;
            marker.scale.z = estimate.height;
            marker.color.r = 0.0f; marker.color.g = 0.6f; marker.color.b = 1.0f;
        } else if (estimate.primitive_type == "FLAT_BOX") {
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = estimate.width;
            marker.scale.y = estimate.depth;
            marker.scale.z = estimate.thickness;
            marker.color.r = 0.2f; marker.color.g = 1.0f; marker.color.b = 0.2f;
        } else {
            return;
        }

        marker.color.a = 1.0f;

        pub_final_marker_.publish(marker);
    }


    bool handleRequest(pcl_package::GetStableEstimate::Request& req,
                        pcl_package::GetStableEstimate::Response& res) {
        if (current_type_ == "UNKNOWN" || buffer_.empty()) {
            res.success = false;
            res.reason = "No object currently detected.";
            return true;
        }

        if (buffer_.size() < kMinSamplesRequired) {
            res.success = false;
            res.reason = "Insufficient samples yet (" + std::to_string(buffer_.size()) +
                        "/" + std::to_string(kMinSamplesRequired) + ").";
            return true;
        }

        std::vector<float> diameters, heights, widths, thicknesses, depths, px, py, pz;
        for (const auto& e : buffer_) {
            diameters.push_back(e.diameter);
            heights.push_back(e.height);
            widths.push_back(e.width);
            thicknesses.push_back(e.thickness);
            depths.push_back(e.depth);
            px.push_back(e.pose.pose.position.x);
            py.push_back(e.pose.pose.position.y);
            pz.push_back(e.pose.pose.position.z);
        }

        res.success = true;
        res.estimate.primitive_type = current_type_;
        res.estimate.valid = true;
        res.estimate.diameter = median(diameters);
        res.estimate.height = median(heights);
        res.estimate.width = median(widths);
        res.estimate.thickness = median(thicknesses);
        res.estimate.depth = median(depths);

        res.estimate.pose.header = buffer_.back().pose.header;
        res.estimate.pose.pose.position.x = median(px);
        res.estimate.pose.pose.position.y = median(py);
        res.estimate.pose.pose.position.z = median(pz);
        res.estimate.pose.pose.orientation = buffer_.back().pose.pose.orientation;

        res.cloud = computeConsensusCloud(res.estimate.pose.header);

        publishFinalMarker(res.estimate);

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PerceptionNode node(nh, pnh);
    ros::spin();
    return 0;
}