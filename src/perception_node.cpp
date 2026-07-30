#include <ros/ros.h>
#include <pcl_package/PrimitiveEstimate.h>
#include <pcl_package/GetStableEstimate.h>
#include <visualization_msgs/Marker.h>
#include <deque>
#include <algorithm>

class PerceptionNode {
public:
    PerceptionNode(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/classification/raw_primitive_estimate", 1,
                             &PerceptionNode::estimateCallback, this);
        service_ = nh.advertiseService("/perception/get_stable_estimate",
                                         &PerceptionNode::handleRequest, this);

        pub_final_marker_ = nh.advertise<visualization_msgs::Marker>("/perception/final_primitive_marker", 1, /*latch=*/true);
    }

private:
    ros::Subscriber sub_;
    ros::ServiceServer service_;
    ros::Publisher pub_final_marker_;

    static constexpr size_t kWindowSize = 20;
    static constexpr size_t kMinSamplesRequired = 10;  // half the window

    std::string current_type_ = "UNKNOWN";
    std::deque<pcl_package::PrimitiveEstimate> buffer_;

    void estimateCallback(const pcl_package::PrimitiveEstimateConstPtr& msg) {
        if (!msg->valid) {
            // No object / no confirmed classification this frame - clear everything
            buffer_.clear();
            current_type_ = "UNKNOWN";
            return;
        }

        if (msg->primitive_type != current_type_) {
            // Classification changed - old samples no longer apply
            buffer_.clear();
            current_type_ = msg->primitive_type;
        }

        buffer_.push_back(*msg);
        if (buffer_.size() > kWindowSize) {
            buffer_.pop_front();
        }
    }

    float median(std::vector<float> values) const {
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        return (n % 2 == 0) ? (values[n/2 - 1] + values[n/2]) / 2.0f : values[n/2];
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

        publishFinalMarker(res.estimate);

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    PerceptionNode node(nh);
    ros::spin();
    return 0;
}