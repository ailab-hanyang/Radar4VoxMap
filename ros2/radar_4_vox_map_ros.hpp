/**
 * @file radar_4_vox_map_ros.hpp
 * @brief ROS2 wrapper for radar_4_vox_map system
 */

#ifndef __RADAR_4_VOX_MAP_ROS_HPP__
#define __RADAR_4_VOX_MAP_ROS_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

#include <ini_handler_cpp/c_ini.hpp>
#include "radar_4_vox_map.hpp"
#include "radar_point_converters.hpp"

class Radar4VoxMapROS : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    Radar4VoxMapROS();

    /**
     * @brief Destructor
     */
    ~Radar4VoxMapROS();

    /**
     * @brief Initialize the node with parameters
     * @return True if initialization succeeds
     */
    bool initialize();

private:
    /**
     * @brief Setup publishers and subscribers
     */
    void setupROSInterface();

    /**
     * @brief Setup field mapping for radar data
     */
    void setupFieldMapping();

    /**
     * @brief Callback for radar point cloud data
     * @param msg PointCloud2 message from radar
     */
    void radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Callback for ground truth pose data
     * @param msg PoseStamped message from ground truth
     */
    void poseCallbackGroundTruth(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Publish all results from the radar_4_vox_map processing
     */
    void publishResults();

    /**
     * @brief Generate and publish voxel visualization
     */
    void publishVoxelMap();

    /**
     * @brief Generate and publish graph visualization
     */
    void publishGraphVisualization();

    /**
     * @brief Generate and update pose visualization
     */
    void updateEstimatedPoseArrayGeo();

    /**
     * @brief Get the current estimated pose as PoseStamped
     * @return Current pose as geometry_msgs::msg::PoseStamped
     */
    geometry_msgs::msg::PoseStamped getEstimatedPoseMsg();

    /**
     * @brief Get the current twist as TwistStamped
     * @return Current twist as geometry_msgs::msg::TwistStamped
     */
    geometry_msgs::msg::TwistStamped getTwistStampedMsg();

    /**
     * @brief Get the TF transform from world to base_link
     * @return TF message
     */
    tf2_msgs::msg::TFMessage getTFMessageMsg();

    /**
     * @brief Convert VoxelBlock to marker array for visualization
     * @return Marker array containing voxel visualization
     */
    visualization_msgs::msg::MarkerArray getVoxelMapMarkerArray();

    /**
     * @brief Convert VoxelBlock to bounding box marker array for visualization
     * @return Marker array containing voxel bounding boxes (CUBE markers)
     */
    visualization_msgs::msg::MarkerArray getVoxelMapBoxArray();

    /**
     * @brief Convert graph elements to marker array for visualization
     * @return Marker array containing graph visualization
     */
    visualization_msgs::msg::MarkerArray getGraphMarkerArray();

    /**
     * @brief Helper to sort eigenvalues and eigenvectors
     * @param eigenvalues Eigen vector of eigenvalues to sort
     * @param eigenvectors Eigen matrix of eigenvectors to sort
     */
    void sortEigenvaluesAndEigenvectors(Eigen::Vector3d& eigenvalues, Eigen::Matrix3d& eigenvectors);

private:
    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ground_truth_pose_sub_;

    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_points_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr estimated_pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_map_marker_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_map_box_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_marker_array_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr graph_vertex_pose_array_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ground_truth_pose_pub_;

    // ROS parameters
    std::string radar_topic_;
    std::string radar_dataset_type_;
    std::string radar_4_vox_map_type_;
    std::string algorithm_ini_path_;
    std::string world_frame_id_;
    std::string base_link_frame_id_;
    bool publish_tf_;
    bool publish_voxel_map_;
    bool publish_graph_;
    bool publish_ground_truth_pose_;
    std::string ground_truth_pose_topic_;

    // Field mapping related variables
    CRadarFieldMapping field_mapping_;

    // radar_4_vox_map related members
    std::shared_ptr<Radar4VoxMapBase> radar_4_vox_map_;
    VoxelRcsMapperConfig vox_map_config_;

    // Processing data
    std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> results_;
    Eigen::Matrix4d radar_pose_;
    rclcpp::Time current_ros_time_;
    Eigen::Vector3d twist_velocity_;
    Eigen::Vector3d twist_angular_velocity_;
    double current_timestamp_;

    // Visualization data
    std::vector<RadarPoint> local_static_map_;
    std::vector<RadarPoint> global_radar_points_;
    std::vector<VoxelHashMap::VoxelBlock> global_voxel_block_;
    std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>, std::vector<SVisEdgeUnary>> graph_elements_;
    std::vector<SVisEdgeUnaryDoppler> vertex_doppler_;

    // Message cache
    sensor_msgs::msg::PointCloud2 local_map_pc2_;
    sensor_msgs::msg::PointCloud2 global_radar_points_pc2_;
    geometry_msgs::msg::PoseArray est_pose_array_msg_;
    visualization_msgs::msg::MarkerArray voxel_map_marker_array_;
    visualization_msgs::msg::MarkerArray voxel_map_box_array_;
    visualization_msgs::msg::MarkerArray graph_marker_array_;
    geometry_msgs::msg::PoseArray graph_node_pose_array_msg_;
    geometry_msgs::msg::PoseStamped est_pose_msg_;
    geometry_msgs::msg::TwistStamped twist_stamped_msg_;
    geometry_msgs::msg::PoseArray ground_truth_pose_array_msg_;
    tf2_msgs::msg::TFMessage tf_message_msg_;

    // Tracking visualization state
    int last_voxel_num_;
    int last_published_vertex_num_;
    int last_published_binary_edge_num_;
    int last_published_unary_edge_num_;
    int last_published_unary_edge_doppler_num_;
};

#endif // __RADAR_4_VOX_MAP_ROS_HPP__
