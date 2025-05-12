/**
 * @file radar_4_vox_map_ros.hpp
 * @brief ROS wrapper for radar_4_vox_map system
 */

#ifndef __RADAR_4_VOX_MAP_ROS_HPP__
#define __RADAR_4_VOX_MAP_ROS_HPP__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <std_msgs/Header.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <ini_handler_cpp/c_ini.hpp>
#include "radar_4_vox_map.hpp"
#include "radar_point_converters.hpp"

class Radar4VoxMapROS {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    Radar4VoxMapROS(ros::NodeHandle& nh);
    
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
    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    /**
     * @brief Publish all results from the radar_4_vox_map processing
     */
    void publishResults();
    
    /**
     * @brief Generate and publish local map from radar_4_vox_map
     */
    void publishLocalMap();
    
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
     * @return Current pose as geometry_msgs::PoseStamped
     */
    geometry_msgs::PoseStamped getEstimatedPoseMsg();
    
    /**
     * @brief Get the TF transform from world to base_link
     * @return TF message
     */
    tf2_msgs::TFMessage getTFMessageMsg();
    
    /**
     * @brief Convert VoxelBlock to marker array for visualization
     * @return Marker array containing voxel visualization
     */
    visualization_msgs::MarkerArray getVoxelMapMarkerArray();
    
    /**
     * @brief Convert VoxelBlock to bounding box array for visualization
     * @return Bounding box array containing voxel boxes
     */
    jsk_recognition_msgs::BoundingBoxArray getVoxelMapBoxArray();
    
    /**
     * @brief Convert graph elements to marker array for visualization
     * @return Marker array containing graph visualization
     */
    visualization_msgs::MarkerArray getGraphMarkerArray();
    
    /**
     * @brief Helper to sort eigenvalues and eigenvectors
     * @param eigenvalues Eigen vector of eigenvalues to sort
     * @param eigenvectors Eigen matrix of eigenvectors to sort
     */
    void sortEigenvaluesAndEigenvectors(Eigen::Vector3d& eigenvalues, Eigen::Matrix3d& eigenvectors);
    
private:
    // ROS related members
    ros::NodeHandle nh_;
    ros::Subscriber radar_sub_;
    
    ros::Publisher local_map_pub_;
    ros::Publisher radar_points_pub_;
    ros::Publisher estimated_pose_array_pub_;
    ros::Publisher voxel_map_marker_array_pub_;
    ros::Publisher voxel_map_box_array_pub_;
    ros::Publisher graph_marker_array_pub_;
    ros::Publisher graph_vertex_pose_array_pub_;
    ros::Publisher estimated_pose_pub_;
    ros::Publisher tf_pub_;
    
    // ROS parameters
    std::string radar_topic_;
    std::string radar_dataset_type_;
    std::string algorithm_ini_path_;
    std::string world_frame_id_;
    std::string radar_frame_id_;
    std::string base_link_frame_id_;
    bool publish_tf_;
    bool publish_voxel_map_;
    bool publish_graph_;
    
    // Field mapping related variables
    CRadarFieldMapping field_mapping_;
    
    // radar_4_vox_map related members
    std::shared_ptr<Radar4VoxMap> radar_4_vox_map_;
    VoxelRcsMapperConfig vox_map_config_;
    
    // Processing data
    std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> results_;
    Eigen::Matrix4d radar_pose_;
    ros::Time current_ros_time_;
    double current_timestamp_;
    
    // Visualization data
    std::vector<RadarPoint> local_static_map_;
    std::vector<RadarPoint> global_radar_points_;
    std::vector<VoxelHashMap::VoxelBlock> global_voxel_block_;
    std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>, std::vector<SVisEdgeUnary>> graph_elements_;
    std::vector<SVisEdgeUnaryDoppler> vertex_doppler_;
    
    // Message cache
    sensor_msgs::PointCloud2 local_map_pc2_;
    sensor_msgs::PointCloud2 global_radar_points_pc2_;
    geometry_msgs::PoseArray est_pose_array_msg_;
    visualization_msgs::MarkerArray voxel_map_marker_array_;
    jsk_recognition_msgs::BoundingBoxArray voxel_map_box_array_;
    visualization_msgs::MarkerArray graph_marker_array_;
    geometry_msgs::PoseArray graph_node_pose_array_msg_;
    geometry_msgs::PoseStamped est_pose_msg_;
    tf2_msgs::TFMessage tf_message_msg_;
    
    // Tracking visualization state
    int last_voxel_num_;
    int last_published_vertex_num_;
    int last_published_binary_edge_num_;
    int last_published_unary_edge_num_;
    int last_published_unary_edge_doppler_num_;
};

#endif // __RADAR_4_VOX_MAP_ROS_HPP__ 