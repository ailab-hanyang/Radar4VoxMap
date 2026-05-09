/**
 * @file radar_4_vox_map_octomap_exporter.hpp
 * @brief Dedicated ROS node for occupied-only OctoMap export from Radar4VoxMap voxel topic
 */

#ifndef __RADAR_4_VOX_MAP_OCTOMAP_EXPORTER_HPP__
#define __RADAR_4_VOX_MAP_OCTOMAP_EXPORTER_HPP__

#include <memory>
#include <string>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <octomap/OcTree.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Radar4VoxMapOctomapExporter {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    explicit Radar4VoxMapOctomapExporter(ros::NodeHandle& nh);

    /**
     * @brief Initialize the node with parameters
     * @return True if initialization succeeds
     */
    bool initialize();

private:
    /**
     * @brief Get transform from the input voxel frame to the configured OctoMap frame
     * @param source_frame Input voxel message frame
     * @param stamp Input voxel message timestamp
     * @param source_to_target Output transform from source frame to target frame
     * @return True when no transform is needed or lookup succeeds
     */
    bool getSourceToTargetTransform(const std::string& source_frame,
                                    const ros::Time& stamp,
                                    tf2::Transform& source_to_target) const;

    /**
     * @brief Build an occupied-only OctoMap from voxel bounding boxes
     * @param msg Voxel bounding box array
     * @return OcTree when conversion succeeds, otherwise nullptr
     */
    std::unique_ptr<octomap::OcTree> buildOctomapFromVoxelBoxes(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) const;

    /**
     * @brief Callback for voxel bounding box messages
     * @param voxel_boxes_msg Voxel bounding box array
     */
    void voxelBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& voxel_boxes_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber voxel_box_sub_;
    ros::Publisher octomap_pub_;
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string input_voxel_box_topic_;

    std::string octomap_export_target_frame_id_;
    std::string octomap_export_topic_name_;
    double octomap_export_resolution_;
    bool octomap_export_binary_;
    bool octomap_export_use_only_static_label_;
};

#endif  // __RADAR_4_VOX_MAP_OCTOMAP_EXPORTER_HPP__
