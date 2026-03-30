/**
 * @file radar_4_vox_map_octomap_exporter.cpp
 * @brief Dedicated ROS node for occupied-only OctoMap export from Radar4VoxMap voxel topic
 */

#include <memory>
#include <string>

#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class Radar4VoxMapOctomapExporter {
public:
    explicit Radar4VoxMapOctomapExporter(ros::NodeHandle& nh)
        : nh_(nh) {}

    bool initialize() {
        nh_.param<std::string>("input_voxel_box_topic", input_voxel_box_topic_, "/radar_4_vox_map/voxel_map_voxel_box_array");

        nh_.param<std::string>("octomap_export/frame_id", octomap_export_frame_id_, "");
        nh_.param<std::string>("octomap_export/topic_name", octomap_export_topic_name_, "/radar_4_vox_map/octomap");
        nh_.param<double>("octomap_export/resolution", octomap_export_resolution_, 0.5);
        nh_.param<bool>("octomap_export/binary", octomap_export_binary_, true);
        nh_.param<bool>("octomap_export/use_only_static_label", octomap_export_use_only_static_label_, true);

        ROS_INFO("Radar4VoxMapOctomapExporter initializing with parameters:");
        ROS_INFO_STREAM("  input_voxel_box_topic: " << input_voxel_box_topic_);
        ROS_INFO_STREAM("  octomap_export/frame_id: " << octomap_export_frame_id_);
        ROS_INFO_STREAM("  octomap_export/topic_name: " << octomap_export_topic_name_);
        ROS_INFO_STREAM("  octomap_export/resolution: " << octomap_export_resolution_);
        ROS_INFO_STREAM("  octomap_export/binary: " << octomap_export_binary_);
        ROS_INFO_STREAM("  octomap_export/use_only_static_label: " << octomap_export_use_only_static_label_);

        if (octomap_export_resolution_ <= 0.0) {
            ROS_WARN("octomap_export/resolution must be > 0. Using fallback 0.5");
            octomap_export_resolution_ = 0.5;
        }

        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>(octomap_export_topic_name_, 1, true);
        voxel_box_sub_ = nh_.subscribe(input_voxel_box_topic_, 5, &Radar4VoxMapOctomapExporter::voxelBoxCallback, this);

        ROS_INFO("Radar4VoxMapOctomapExporter initialized");
        return true;
    }

private:
    std::unique_ptr<octomap::OcTree> buildOctomapFromVoxelBoxes(
        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) const {
        auto tree = std::make_unique<octomap::OcTree>(octomap_export_resolution_);

        for (const auto& box : msg->boxes) {
            if (octomap_export_use_only_static_label_ && box.label == 0) {
                continue;
            }

            tree->updateNode(
                octomap::point3d(static_cast<float>(box.pose.position.x),
                                 static_cast<float>(box.pose.position.y),
                                 static_cast<float>(box.pose.position.z)),
                true);
        }

        tree->updateInnerOccupancy();
        return tree;
    }

    void voxelBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& voxel_boxes_msg) {
        auto octree = buildOctomapFromVoxelBoxes(voxel_boxes_msg);

        octomap_msgs::Octomap octomap_msg;
        octomap_msg.header.stamp = ros::Time::now();
        octomap_msg.header.frame_id = octomap_export_frame_id_.empty() ? voxel_boxes_msg->header.frame_id : octomap_export_frame_id_;

        bool converted = false;
        if (octomap_export_binary_) {
            converted = octomap_msgs::binaryMapToMsg(*octree, octomap_msg);
        } else {
            converted = octomap_msgs::fullMapToMsg(*octree, octomap_msg);
        }

        if (!converted) {
            ROS_WARN("Failed to convert OcTree to octomap_msgs/Octomap");
            return;
        }

        octomap_pub_.publish(octomap_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber voxel_box_sub_;
    ros::Publisher octomap_pub_;

    std::string input_voxel_box_topic_;

    std::string octomap_export_frame_id_;
    std::string octomap_export_topic_name_;
    double octomap_export_resolution_;
    bool octomap_export_binary_;
    bool octomap_export_use_only_static_label_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_4_vox_map_octomap_exporter");
    ros::NodeHandle nh("~");

    Radar4VoxMapOctomapExporter exporter(nh);
    if (!exporter.initialize()) {
        ROS_ERROR("Failed to initialize Radar4VoxMapOctomapExporter");
        return 1;
    }

    ros::spin();
    return 0;
}
