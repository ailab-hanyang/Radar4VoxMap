/**
 * @file radar_4_vox_map_octomap_exporter.cpp
 * @brief Dedicated ROS node for occupied-only OctoMap export from Radar4VoxMap voxel topic
 */

#include "radar_4_vox_map_octomap_exporter.hpp"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

Radar4VoxMapOctomapExporter::Radar4VoxMapOctomapExporter(ros::NodeHandle& nh)
    : nh_(nh) {}

bool Radar4VoxMapOctomapExporter::initialize() {
    nh_.param<std::string>("input_voxel_box_topic", input_voxel_box_topic_,
                           "/radar_4_vox_map/voxel_map_voxel_box_array");

    nh_.param<std::string>("octomap_export/target_frame_id", octomap_export_target_frame_id_, "");
    nh_.param<std::string>("octomap_export/topic_name", octomap_export_topic_name_, "/radar_4_vox_map/octomap");
    nh_.param<double>("octomap_export/resolution", octomap_export_resolution_, 0.5);
    nh_.param<bool>("octomap_export/binary", octomap_export_binary_, true);
    nh_.param<bool>("octomap_export/use_only_static_label", octomap_export_use_only_static_label_, true);

    ROS_INFO("Radar4VoxMapOctomapExporter initializing with parameters:");
    ROS_INFO_STREAM("  input_voxel_box_topic: " << input_voxel_box_topic_);
    ROS_INFO_STREAM("  octomap_export/target_frame_id: " << octomap_export_target_frame_id_);
    ROS_INFO_STREAM("  octomap_export/topic_name: " << octomap_export_topic_name_);
    ROS_INFO_STREAM("  octomap_export/resolution: " << octomap_export_resolution_);
    ROS_INFO_STREAM("  octomap_export/binary: " << octomap_export_binary_);
    ROS_INFO_STREAM("  octomap_export/use_only_static_label: " << octomap_export_use_only_static_label_);

    if (octomap_export_resolution_ <= 0.0) {
        ROS_WARN("octomap_export/resolution must be > 0. Using fallback 0.5");
        octomap_export_resolution_ = 0.5;
    }

    if (!octomap_export_target_frame_id_.empty()) {
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    }

    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>(octomap_export_topic_name_, 1, true);
    voxel_box_sub_ = nh_.subscribe(input_voxel_box_topic_, 5, &Radar4VoxMapOctomapExporter::voxelBoxCallback, this);

    ROS_INFO("Radar4VoxMapOctomapExporter initialized");
    return true;
}

bool Radar4VoxMapOctomapExporter::getSourceToTargetTransform(
    const std::string& source_frame,
    const ros::Time& stamp,
    tf2::Transform& source_to_target) const {
    source_to_target.setIdentity();

    if (octomap_export_target_frame_id_.empty() || source_frame == octomap_export_target_frame_id_) {
        return true;
    }

    if (source_frame.empty()) {
        ROS_WARN_THROTTLE(5.0, "Cannot transform OctoMap export: input voxel box frame_id is empty");
        return false;
    }

    try {
        const auto transform_msg = tf_buffer_.lookupTransform(octomap_export_target_frame_id_,
                                                              source_frame,
                                                              stamp,
                                                              ros::Duration(0.05));

        const auto& translation_msg = transform_msg.transform.translation;
        const auto& rotation_msg = transform_msg.transform.rotation;
        const tf2::Vector3 translation(translation_msg.x, translation_msg.y, translation_msg.z);
        const tf2::Quaternion rotation(rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w);
        source_to_target = tf2::Transform(rotation, translation);
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM_THROTTLE(5.0,
                                 "Failed to transform OctoMap export from '" << source_frame
                                 << "' to '" << octomap_export_target_frame_id_
                                 << "': " << ex.what());
        return false;
    }
}

std::unique_ptr<octomap::OcTree> Radar4VoxMapOctomapExporter::buildOctomapFromVoxelBoxes(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) const {
    auto tree = std::make_unique<octomap::OcTree>(octomap_export_resolution_);
    tf2::Transform source_to_target;
    if (!getSourceToTargetTransform(msg->header.frame_id, msg->header.stamp, source_to_target)) {
        return nullptr;
    }

    for (const auto& box : msg->boxes) {
        if (octomap_export_use_only_static_label_ && box.label == 0) {
            continue;
        }

        const tf2::Vector3 point_in_source(box.pose.position.x,
                                           box.pose.position.y,
                                           box.pose.position.z);
        const tf2::Vector3 point_for_octomap = source_to_target * point_in_source;
        tree->updateNode(
            octomap::point3d(static_cast<float>(point_for_octomap.x()),
                             static_cast<float>(point_for_octomap.y()),
                             static_cast<float>(point_for_octomap.z())),
            true);
    }

    tree->updateInnerOccupancy();
    return tree;
}

void Radar4VoxMapOctomapExporter::voxelBoxCallback(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& voxel_boxes_msg) {
    auto octree = buildOctomapFromVoxelBoxes(voxel_boxes_msg);
    if (!octree) {
        return;
    }

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = voxel_boxes_msg->header.stamp;
    if (!octomap_export_target_frame_id_.empty()) {
        octomap_msg.header.frame_id = octomap_export_target_frame_id_;
    } else {
        octomap_msg.header.frame_id = voxel_boxes_msg->header.frame_id;
    }

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
