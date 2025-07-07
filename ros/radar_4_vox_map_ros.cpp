/**
 * @file radar_4_vox_map_ros.cpp
 * @brief ROS wrapper implementation for radar_4_vox_map system
 */

#include "radar_4_vox_map_ros.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>

Radar4VoxMapROS::Radar4VoxMapROS(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize default values
    radar_pose_ = Eigen::Matrix4d::Identity();
    current_ros_time_ = ros::Time(0, 0);
    current_timestamp_ = 0.0;
    
    last_voxel_num_ = 0;
    last_published_vertex_num_ = 0;
    last_published_binary_edge_num_ = 0;
    last_published_unary_edge_num_ = 0;
    last_published_unary_edge_doppler_num_ = 0;
    
    // Initialize radar_4_vox_map core
    radar_4_vox_map_ = std::make_shared<Radar4VoxMap>();
}

Radar4VoxMapROS::~Radar4VoxMapROS() {
    ROS_INFO("Radar4VoxMapROS is being destroyed");
}

bool Radar4VoxMapROS::initialize() {
    // Load parameters
    nh_.param<std::string>("radar_topic", radar_topic_, "/radar/points");
    nh_.param<std::string>("radar_dataset_type", radar_dataset_type_, "vod");
    nh_.param<std::string>("radar_4_vox_map_type", radar_4_vox_map_type_, "3d");
    nh_.param<std::string>("algorithm_ini_path", algorithm_ini_path_, "config/radar_4_vox_map.ini");
    nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
    nh_.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
    nh_.param<bool>("publish_tf", publish_tf_, true);
    nh_.param<bool>("publish_voxel_map", publish_voxel_map_, true);
    nh_.param<bool>("publish_graph", publish_graph_, true);
    nh_.param<bool>("publish_ground_truth_pose", publish_ground_truth_pose_, false);
    nh_.param<std::string>("ground_truth_pose_topic", ground_truth_pose_topic_, "/pose/local");
    
    ROS_INFO("Radar4VoxMapROS initializing with parameters:");
    ROS_INFO_STREAM("  radar_topic: " << radar_topic_);
    ROS_INFO_STREAM("  radar_dataset_type: " << radar_dataset_type_);
    ROS_INFO_STREAM("  radar_4_vox_map_type: " << radar_4_vox_map_type_);
    ROS_INFO_STREAM("  algorithm_ini_path: " << algorithm_ini_path_);
    ROS_INFO_STREAM("  world_frame_id: " << world_frame_id_);
    ROS_INFO_STREAM("  base_link_frame_id: " << base_link_frame_id_);
    ROS_INFO_STREAM("  publish_tf: " << publish_tf_);
    ROS_INFO_STREAM("  publish_voxel_map: " << publish_voxel_map_);
    ROS_INFO_STREAM("  publish_graph: " << publish_graph_);
    ROS_INFO_STREAM("  publish_ground_truth_pose: " << publish_ground_truth_pose_);
    ROS_INFO_STREAM("  ground_truth_pose_topic: " << ground_truth_pose_topic_);
    
    // Setup ROS interface
    setupROSInterface();
    
    // Load algorithm configuration from INI file
    std::string pkg_path = ros::package::getPath("radar_4_vox_map");
    std::string ini_dir = algorithm_ini_path_;
    std::string full_ini_path = pkg_path + "/" + ini_dir;
    
    CINI_H ini_handler;
    ini_handler.Init(full_ini_path.c_str());
    
    ROS_INFO_STREAM("Loading configuration from: " << full_ini_path);
    
    if (ini_handler.IsFileUpdated()) {
        // Load TBB settings
        ini_handler.ParseConfig("code", "use_tbb", vox_map_config_.use_tbb);
        ini_handler.ParseConfig("code", "max_thread_num", vox_map_config_.max_thread_num);
        
        // Load sensor settings
        if (!ini_handler.ParseConfig("sensor", "radar_range_variance_m", vox_map_config_.radar_range_variance_m) ||
            !ini_handler.ParseConfig("sensor", "radar_azimuth_variance_deg", vox_map_config_.radar_azimuth_variance_deg) ||
            !ini_handler.ParseConfig("sensor", "radar_elevation_variance_deg", vox_map_config_.radar_elevation_variance_deg)) {
            ROS_ERROR("Failed to load sensor parameters");
            return false;
        }
        
        // Load optimization settings
        if (!ini_handler.ParseConfig("optimization", "optimization_num", vox_map_config_.optimization_num) ||
            !ini_handler.ParseConfig("optimization", "optimization_algorithm_type", vox_map_config_.optimization_algorithm_type)) {
            ROS_ERROR("Failed to load optimization parameters");
            return false;
        }
        
        ini_handler.ParseConfig("optimization", "run_optimization", vox_map_config_.run_optimization);
        ini_handler.ParseConfig("optimization", "icp_all_node", vox_map_config_.icp_all_node);
        ini_handler.ParseConfig("optimization", "use_doppler_unary_edge", vox_map_config_.use_doppler_unary_edge);
        ini_handler.ParseConfig("optimization", "node_erase_distance", vox_map_config_.node_erase_distance);
        ini_handler.ParseConfig("optimization", "vertex_num_max", vox_map_config_.vertex_num_max);
        ini_handler.ParseConfig("optimization", "optimization_vertex_num_max", vox_map_config_.optimization_vertex_num_max);
        ini_handler.ParseConfig("optimization", "fixed_vertex_to_map_keyframe_distance", vox_map_config_.fixed_vertex_to_map_keyframe_distance);
        ini_handler.ParseConfig("optimization", "edge_unary_dicp_std", vox_map_config_.edge_unary_dicp_std);
        ini_handler.ParseConfig("optimization", "edge_binary_cv_std", vox_map_config_.edge_binary_cv_std);
        
        // Load ICP settings
        int i_icp_method;
        if (!ini_handler.ParseConfig("icp", "icp_method", i_icp_method) ||
            !ini_handler.ParseConfig("icp", "icp_max_iteration", vox_map_config_.icp_max_iteration) ||
            !ini_handler.ParseConfig("icp", "icp_termination_threshold_m", vox_map_config_.icp_termination_threshold_m) ||
            !ini_handler.ParseConfig("icp", "doppler_trans_lambda", vox_map_config_.doppler_trans_lambda) ||
            !ini_handler.ParseConfig("icp", "icp_use_doppler", vox_map_config_.icp_use_doppler)) {
            ROS_ERROR("Failed to load ICP parameters");
            return false;
        }
        vox_map_config_.icp_method = IcpMethod(i_icp_method);
        
        ini_handler.ParseConfig("icp", "static_residual_geo", vox_map_config_.static_residual_geo);
        ini_handler.ParseConfig("icp", "static_residual_vel", vox_map_config_.static_residual_vel);
        ini_handler.ParseConfig("icp", "scan_voxel_size", vox_map_config_.scan_voxel_size);
        
        // Load voxel map settings
        if (!ini_handler.ParseConfig("voxel_map", "voxel_size", vox_map_config_.voxel_size) ||
            !ini_handler.ParseConfig("voxel_map", "voxel_map_max_distance", vox_map_config_.voxel_map_max_distance) ||
            !ini_handler.ParseConfig("voxel_map", "voxel_max_point_num", vox_map_config_.voxel_max_point_num) ||
            !ini_handler.ParseConfig("voxel_map", "voxel_search_level", vox_map_config_.voxel_search_level)) {
            ROS_ERROR("Failed to load voxel map parameters");
            return false;
        }
        
        ini_handler.ParseConfig("voxel_map", "voxel_map_use_rcs", vox_map_config_.voxel_map_use_rcs);
        ini_handler.ParseConfig("voxel_map", "voxel_map_use_range_weight", vox_map_config_.voxel_map_use_range_weight);
        
        // Load adaptive threshold settings
        if (!ini_handler.ParseConfig("adaptive_threshold", "initial_trans_threshold", vox_map_config_.initial_trans_threshold) ||
            !ini_handler.ParseConfig("adaptive_threshold", "initial_vel_threshold", vox_map_config_.initial_vel_threshold) ||
            !ini_handler.ParseConfig("adaptive_threshold", "min_motion_threshold", vox_map_config_.min_motion_threshold)) {
            ROS_ERROR("Failed to load adaptive threshold parameters");
            return false;
        }
        
        // Load output settings
        if (!ini_handler.ParseConfig("output", "local_map_property", vox_map_config_.local_map_property) ||
            !ini_handler.ParseConfig("output", "voxel_visualize_min_point", vox_map_config_.voxel_visualize_min_point)) {
            ROS_ERROR("Failed to load output parameters");
            return false;
        }
        
        ini_handler.ParseConfig("output", "voxel_visualize_only_static", vox_map_config_.voxel_visualize_only_static);

        // Load virtual gravity align settings
        ini_handler.ParseConfig("virtual_gravity_align", "virtual_gravity_align", vox_map_config_.virtual_gravity_align);
        ini_handler.ParseConfig("virtual_gravity_align", "virtual_gravity_align_information", vox_map_config_.virtual_gravity_align_information);
    } else {
        ROS_ERROR("Failed to load INI file: %s", full_ini_path.c_str());
        return false;
    }
    
    // Initialize radar_4_vox_map
    if (!radar_4_vox_map_) {
        ROS_ERROR("Failed to create radar_4_vox_map instance");
        return false;
    }

    if (radar_4_vox_map_type_ == "3d") {
        radar_4_vox_map_ = std::make_shared<Radar4VoxMap>();
    } else if (radar_4_vox_map_type_ == "2d") {
        radar_4_vox_map_ = std::make_shared<Radar4VoxMap2D>();
    }
    
    radar_4_vox_map_->Init(vox_map_config_);
    ROS_INFO("radar_4_vox_map initialized successfully");
    
    return true;
}

void Radar4VoxMapROS::setupROSInterface() {
    // Setup publishers with queue size 5
    local_map_pub_               = nh_.advertise<sensor_msgs::PointCloud2>("/radar_4_vox_map/radar_local_map", 5);
    radar_points_pub_            = nh_.advertise<sensor_msgs::PointCloud2>("/radar_4_vox_map/radar_points", 5);
    estimated_pose_array_pub_    = nh_.advertise<geometry_msgs::PoseArray>("/radar_4_vox_map/estimated_pose_array", 5);
    voxel_map_marker_array_pub_  = nh_.advertise<visualization_msgs::MarkerArray>("/radar_4_vox_map/voxel_map_marker_array", 5);
    voxel_map_box_array_pub_     = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/radar_4_vox_map/voxel_map_voxel_box_array", 5);
    graph_marker_array_pub_      = nh_.advertise<visualization_msgs::MarkerArray>("/radar_4_vox_map/graph_marker_array", 5);
    graph_vertex_pose_array_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/radar_4_vox_map/graph_vertex_pose_array", 5);
    estimated_pose_pub_          = nh_.advertise<geometry_msgs::PoseStamped>("/radar_4_vox_map/estimated_pose", 5);
    twist_pub_                   = nh_.advertise<geometry_msgs::TwistStamped>("/radar_4_vox_map/twist", 5);
    ground_truth_pose_pub_       = nh_.advertise<geometry_msgs::PoseArray>("/radar_4_vox_map/ground_truth_pose_array", 5);
    tf_pub_                      = nh_.advertise<tf2_msgs::TFMessage>("/tf", 10);

    // Setup subscriber with queue size 10
    radar_sub_ = nh_.subscribe(radar_topic_, 10, &Radar4VoxMapROS::radarCallback, this);
    ground_truth_pose_sub_ = nh_.subscribe(ground_truth_pose_topic_, 10, &Radar4VoxMapROS::poseCallbackGroundTruth, this);
    
    ROS_INFO("ROS interface setup completed");
}

void Radar4VoxMapROS::radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Extract timestamp
    current_ros_time_ = msg->header.stamp;
    current_timestamp_ = static_cast<double>(current_ros_time_.toNSec()) / 1e9;
    
    // Convert from ROS PointCloud2 to SRadarPoint format
    RadarPointCloudPtr radar_points_ptr;
    if (radar_dataset_type_ == "vod" || 
        radar_dataset_type_ == "VOD" || 
        radar_dataset_type_ == "view_of_delft" || 
        radar_dataset_type_ == "ViewOfDelft") {
        radar_points_ptr = CRadarFieldMapping::TransformFromViewOfDelft(*msg);
    }
    else if (radar_dataset_type_ == "afi910" || 
             radar_dataset_type_ == "AFI910") {
        radar_points_ptr = CRadarFieldMapping::TransformFromAfi910(*msg);
    }
    else if (radar_dataset_type_ == "mfi920" || 
             radar_dataset_type_ == "MFI920") {
        radar_points_ptr = CRadarFieldMapping::TransformFromMfi920(*msg);
    }
    else if (radar_dataset_type_ == "ars548" || 
             radar_dataset_type_ == "ARS548") {
        radar_points_ptr = CRadarFieldMapping::TransformFromArs548(*msg);
    }
    else if (radar_dataset_type_ == "custom" || 
             radar_dataset_type_ == "Custom") {
        radar_points_ptr = CRadarFieldMapping::Custom(*msg);
    }
    else {
        ROS_ERROR("Unsupported radar dataset type: %s", radar_dataset_type_.c_str());
        radar_points_ptr = nullptr;
        return;
    }
    
    // Process points with radar_4_vox_map
    results_ = radar_4_vox_map_->RunCore(*radar_points_ptr, current_timestamp_);
    
    // Publish results
    publishResults();
}

void Radar4VoxMapROS::poseCallbackGroundTruth(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Get current pose
    Eigen::Vector3d translation = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rotation = q.toRotationMatrix();

    Eigen::Affine3d pose = Eigen::Affine3d();
    pose.translation() = translation;
    pose.linear() = rotation;

    // Get relative pose with camera to radar calibration
    static bool is_first_msg = true;
    static Eigen::Affine3d first_pose;
    static Eigen::Affine3d first_pose_inv;
    static Eigen::Affine3d estimated_pose_to_radar_calibration; 

    if (is_first_msg) {
        is_first_msg = false;

        // Camera to Radar - Tr_velo_to_cam: -0.013857 -0.9997468 0.01772762 0.05283124 0.10934269 -0.01913807 -0.99381983 0.98100483 0.99390751 -0.01183297 0.1095802 1.44445002
        Eigen::Vector3d translation_radar = Eigen::Vector3d(3.44445002, 0.05283124, -0.98100483);
        // Eigen::Vector3d translation_radar = Eigen::Vector3d(1.44445002, 0.05283124, -0.98100483);
        Eigen::Matrix3d rotation_radar = Eigen::Matrix3d::Identity();
        rotation_radar << -0.013857, -0.9997468, 0.01772762,
                          0.10934269, -0.01913807, -0.99381983,
                          0.99390751, -0.01183297, 0.1095802;
        estimated_pose_to_radar_calibration = Eigen::Affine3d();
        estimated_pose_to_radar_calibration.translation() = translation_radar;
        estimated_pose_to_radar_calibration.linear() = rotation_radar;

        first_pose = pose * estimated_pose_to_radar_calibration;
        first_pose_inv = first_pose.inverse();
    }

    // Get relative pose with camera to radar calibration
    Eigen::Affine3d relative_pose =  first_pose_inv * pose * estimated_pose_to_radar_calibration;
    Eigen::Quaterniond q_relative(relative_pose.rotation());

    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = relative_pose.translation().x();
    pose_msg.position.y = relative_pose.translation().y();
    pose_msg.position.z = relative_pose.translation().z();
    pose_msg.orientation.x = q_relative.x();
    pose_msg.orientation.y = q_relative.y();
    pose_msg.orientation.z = q_relative.z();
    pose_msg.orientation.w = q_relative.w();

    // push back to ground_truth_pose_array_msg_
    ground_truth_pose_array_msg_.header = msg->header;
    ground_truth_pose_array_msg_.header.frame_id = world_frame_id_;
    ground_truth_pose_array_msg_.poses.push_back(pose_msg);    
}

void Radar4VoxMapROS::publishResults() {
    // Extract local points and estimated pose
    std::vector<SRadarPoint> frame_local = std::get<0>(results_.first);
    radar_pose_ = std::get<1>(results_.first);
    
    // Transform points to global frame
    std::vector<SRadarPoint> frame_global;
    frame_global.resize(frame_local.size());
    TransformPoints(radar_pose_, frame_local, frame_global);
    
    global_radar_points_ = ConvertSRadarPointsToRadarPoints(frame_global);
    
    // Get local map based on configuration
    if (vox_map_config_.local_map_property == 0) {
        local_static_map_ = ConvertSRadarPointsToRadarPoints(radar_4_vox_map_->LocalMap());
    } else {
        local_static_map_ = ConvertSRadarPointsToRadarPoints(radar_4_vox_map_->StaticLocalMap());
    }
    
    // Get graph elements for visualization
    graph_elements_ = radar_4_vox_map_->GetAllGraphElements();
    
    // Skip publishing if no vertices are available yet
    if (std::get<0>(graph_elements_).size() == 0) {
        ROS_WARN("No graph vertices available yet, skipping visualization");
        return;
    }
    
    vertex_doppler_ = radar_4_vox_map_->GetVertexDopplerVel();
    auto motion = radar_4_vox_map_->GetMotion();
    std::vector<double> motion_vec = motion.first;
    twist_velocity_ = Eigen::Vector3d(motion_vec[0], motion_vec[1], motion_vec[2]);
    twist_angular_velocity_ = Eigen::Vector3d(motion_vec[3], motion_vec[4], motion_vec[5]);
    
    // Update and publish messages
    updateEstimatedPoseArrayGeo();
    publishGraphVisualization();
    
    est_pose_msg_ = getEstimatedPoseMsg();
    twist_stamped_msg_ = getTwistStampedMsg();
    
    // Convert to PointCloud2 and publish
    std_msgs::Header header;
    header.frame_id = world_frame_id_;
    header.stamp = current_ros_time_;
    
    sensor_msgs::PointCloud2 local_map_pc2 = *CRadarFieldMapping::toROSPointCloud2(local_static_map_, header);
    sensor_msgs::PointCloud2 radar_points_pc2 = *CRadarFieldMapping::toROSPointCloud2(global_radar_points_, header);
    
    local_map_pub_.publish(local_map_pc2);
    radar_points_pub_.publish(radar_points_pc2);
    estimated_pose_array_pub_.publish(est_pose_array_msg_);
    estimated_pose_pub_.publish(est_pose_msg_);
    twist_pub_.publish(twist_stamped_msg_);

    if (publish_ground_truth_pose_) {
        if (ground_truth_pose_array_msg_.poses.size() > 0) {
            ground_truth_pose_pub_.publish(ground_truth_pose_array_msg_);
        }
    }
    
    // Publish TF if enabled
    if (publish_tf_) {
        tf_message_msg_ = getTFMessageMsg();
        tf_pub_.publish(tf_message_msg_);
    }
    
    // Publish voxel map if enabled
    if (publish_voxel_map_) {
        publishVoxelMap();
    }
    
    ROS_INFO("Published radar_4_vox_map results");
}

void Radar4VoxMapROS::publishVoxelMap() {
    // Get voxel blocks from map
    if (radar_4_vox_map_type_ == "2d") {
        global_voxel_block_.clear();
        auto global_voxel_block_2d = radar_4_vox_map_->GetAllVoxelFromMap2D();
        for ( auto& voxel : global_voxel_block_2d ) {
            VoxelHashMap::VoxelBlock voxel_block;
            voxel_block.points = voxel.points;
            voxel_block.covariance.cov = Eigen::Matrix3d::Zero();
            voxel_block.covariance.cov.block<2, 2>(0, 0) = voxel.covariance.cov;
            voxel_block.covariance.cov(2, 2) = 0.0001;
            voxel_block.covariance.mean = Eigen::Vector3d::Zero();
            voxel_block.covariance.mean.head<2>() = voxel.covariance.mean;
            voxel_block.covariance.mean(2) = 0.0;
            voxel_block.voxel_center.head<2>() = voxel.voxel_center;
            voxel_block.voxel_center(2) = 0.0;
            voxel_block.hash_value = voxel.hash_value;
            voxel_block.ref_rcs = voxel.ref_rcs;
            voxel_block.is_static = voxel.is_static;
            voxel_block.max_num_points = voxel.max_num_points;
            voxel_block.is_changed = voxel.is_changed;
            global_voxel_block_.push_back(voxel_block);
        }
    }
    else {
        global_voxel_block_ = radar_4_vox_map_->GetAllVoxelFromMap();
    }
    
    // Convert to visualization messages
    voxel_map_marker_array_ = getVoxelMapMarkerArray();
    voxel_map_box_array_ = getVoxelMapBoxArray();
    
    // Publish visualization
    voxel_map_marker_array_pub_.publish(voxel_map_marker_array_);
    voxel_map_box_array_pub_.publish(voxel_map_box_array_);
}

void Radar4VoxMapROS::publishGraphVisualization() {
    if (publish_graph_) {
        graph_marker_array_ = getGraphMarkerArray();
        graph_marker_array_pub_.publish(graph_marker_array_);
        graph_vertex_pose_array_pub_.publish(graph_node_pose_array_msg_);
    }
}

void Radar4VoxMapROS::updateEstimatedPoseArrayGeo() {
    // Extract position and orientation from transform
    geometry_msgs::Pose est_pose_msg;
    est_pose_msg.position.x = radar_pose_(0, 3);
    est_pose_msg.position.y = radar_pose_(1, 3);
    est_pose_msg.position.z = radar_pose_(2, 3);
    
    // Convert rotation matrix to quaternion
    Eigen::Matrix3d rotation_matrix = radar_pose_.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rotation_matrix);
    est_pose_msg.orientation.x = quat.x();
    est_pose_msg.orientation.y = quat.y();
    est_pose_msg.orientation.z = quat.z();
    est_pose_msg.orientation.w = quat.w();
    
    // Set message header
    est_pose_array_msg_.header.frame_id = world_frame_id_;
    est_pose_array_msg_.header.stamp = current_ros_time_;
    est_pose_array_msg_.poses.push_back(est_pose_msg);
}

geometry_msgs::TwistStamped Radar4VoxMapROS::getTwistStampedMsg() {
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = base_link_frame_id_;
    twist_msg.header.stamp = current_ros_time_;
    twist_msg.twist.linear.x = twist_velocity_.x();
    twist_msg.twist.linear.y = twist_velocity_.y();
    twist_msg.twist.linear.z = twist_velocity_.z();
    twist_msg.twist.angular.x = twist_angular_velocity_.x();
    twist_msg.twist.angular.y = twist_angular_velocity_.y();
    twist_msg.twist.angular.z = twist_angular_velocity_.z();
    return twist_msg;
}

geometry_msgs::PoseStamped Radar4VoxMapROS::getEstimatedPoseMsg() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = current_ros_time_;
    
    // Use the first pose from the pose array if available
    if (!est_pose_array_msg_.poses.empty()) {
        pose_msg.pose = est_pose_array_msg_.poses.back();
    } else {
        // Default pose at origin if no poses are available
        pose_msg.pose.orientation.w = 1.0;
    }
    
    return pose_msg;
}

tf2_msgs::TFMessage Radar4VoxMapROS::getTFMessageMsg() {
    geometry_msgs::TransformStamped transform_stamped;
    tf2_msgs::TFMessage tf_msg;
    
    transform_stamped.header.stamp = current_ros_time_;
    transform_stamped.header.frame_id = world_frame_id_;
    transform_stamped.child_frame_id = base_link_frame_id_;
    
    // Use radar_pose_ directly for TF translation and rotation
    Eigen::Vector3d translation = radar_pose_.block<3,1>(0,3);
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    Eigen::Matrix3d rot = radar_pose_.block<3,3>(0,0);
    Eigen::Quaterniond quat(rot);
    transform_stamped.transform.rotation.x = quat.x();
    transform_stamped.transform.rotation.y = quat.y();
    transform_stamped.transform.rotation.z = quat.z();
    transform_stamped.transform.rotation.w = quat.w();
    
    tf_msg.transforms.push_back(transform_stamped);
    return tf_msg;
}

visualization_msgs::MarkerArray Radar4VoxMapROS::getVoxelMapMarkerArray() {
    visualization_msgs::MarkerArray marker_array;
    
    int cur_voxel_num = global_voxel_block_.size();
    
    // Calculate max & min RCS in all voxels
    double max_rcs = -std::numeric_limits<double>::max();
    double min_rcs = std::numeric_limits<double>::max();
    double max_point_num = -std::numeric_limits<double>::max();
    double min_point_num = std::numeric_limits<double>::max();
    
    for (int i = 0; i < cur_voxel_num; ++i) {
        const auto& voxel = global_voxel_block_[i];
        
        // Skip voxels that don't meet visualization criteria
        if (voxel.points.size() < vox_map_config_.voxel_visualize_min_point ||
            (!voxel.is_static && vox_map_config_.voxel_visualize_only_static)) {
            continue;
        }
        
        // Update min/max values
        if (voxel.ref_rcs > max_rcs) max_rcs = voxel.ref_rcs;
        if (voxel.ref_rcs < min_rcs) min_rcs = voxel.ref_rcs;
        if (voxel.points.size() > max_point_num) max_point_num = voxel.points.size();
        if (voxel.points.size() < min_point_num) min_point_num = voxel.points.size();
    }
    
    for (int i = 0; i < cur_voxel_num; ++i) {
        // Create covariance marker
        visualization_msgs::Marker cov_marker;
        visualization_msgs::Marker text_marker;
        
        const auto& voxel = global_voxel_block_[i];
        
        // Skip voxels that don't meet visualization criteria
        if (voxel.points.size() < vox_map_config_.voxel_visualize_min_point ||
            (!voxel.is_static && vox_map_config_.voxel_visualize_only_static)) {
            // Add delete markers for previously existing markers
            cov_marker.header.frame_id = world_frame_id_;
            cov_marker.header.stamp = current_ros_time_;
            cov_marker.ns = "covariance";
            cov_marker.id = i * 2;
            cov_marker.type = visualization_msgs::Marker::SPHERE;
            cov_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(cov_marker);
            
            text_marker.header.frame_id = world_frame_id_;
            text_marker.header.stamp = current_ros_time_;
            text_marker.ns = "covariance_info";
            text_marker.id = i * 2 + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(text_marker);
            continue;
        }
        // Setup covariance ellipsoid marker
        cov_marker.header.frame_id = world_frame_id_;
        cov_marker.header.stamp = current_ros_time_;
        cov_marker.ns = "covariance";
        cov_marker.id = i * 2;
        cov_marker.type = visualization_msgs::Marker::CYLINDER;
        cov_marker.action = visualization_msgs::Marker::ADD;
        
        // Set position to mean
        Eigen::Matrix3d dist_cov = voxel.covariance.cov;
        Eigen::Vector3d dist_mean = voxel.covariance.mean;
        cov_marker.pose.position.x = dist_mean(0);
        cov_marker.pose.position.y = dist_mean(1);
        cov_marker.pose.position.z = dist_mean(2);
        
        // Calculate eigenvalues and eigenvectors for oriented ellipsoid
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> dist_cov_eig(dist_cov);
        Eigen::Vector3d dist_cov_eigenvalues = dist_cov_eig.eigenvalues();
        Eigen::Matrix3d dist_cov_eigenvectors = dist_cov_eig.eigenvectors();
        
        // Sort eigenvalues and eigenvectors
        sortEigenvaluesAndEigenvectors(dist_cov_eigenvalues, dist_cov_eigenvectors);
        
        // Set orientation using eigenvectors
        Eigen::Quaterniond dist_cov_quaternion(dist_cov_eigenvectors);
        cov_marker.pose.orientation.x = dist_cov_quaternion.x();
        cov_marker.pose.orientation.y = dist_cov_quaternion.y();
        cov_marker.pose.orientation.z = dist_cov_quaternion.z();
        cov_marker.pose.orientation.w = dist_cov_quaternion.w();
        
        // Set scale using eigenvalues (scaled to 3-sigma)
        cov_marker.scale.x = 3 * sqrt(dist_cov_eigenvalues(0));
        cov_marker.scale.y = 3 * sqrt(dist_cov_eigenvalues(1));
        cov_marker.scale.z = 3 * sqrt(dist_cov_eigenvalues(2));
        
        // Set color based on RCS (red-yellow-green-cyan-blue color scheme)
        std_msgs::ColorRGBA rcs_color;
        if (max_rcs != min_rcs) {
            double rcs_color_ratio = (voxel.ref_rcs - min_rcs) / (max_rcs - min_rcs);
            if (rcs_color_ratio < 0.5) {
                rcs_color.r = 1.0 - 2 * rcs_color_ratio;
                rcs_color.g = 2 * rcs_color_ratio;
                rcs_color.b = 0.0;
            } else {
                rcs_color.r = 0.0;
                rcs_color.g = 1.0 - 2 * (rcs_color_ratio - 0.5);
                rcs_color.b = 2 * (rcs_color_ratio - 0.5);
            }
        } else {
            rcs_color.r = rcs_color.g = rcs_color.b = 1.0;
        }
        
        // Set alpha based on number of points
        if (max_point_num != min_point_num) {
            rcs_color.a = (voxel.points.size() - min_point_num) / (max_point_num - min_point_num) * 0.5 + 0.5;
        } else {
            rcs_color.a = 1.0;
        }
        
        cov_marker.color = rcs_color;
        marker_array.markers.push_back(cov_marker);
        
        // // Add text marker with voxel information
        // text_marker.header.frame_id = world_frame_id_;
        // text_marker.header.stamp = current_ros_time_;
        // text_marker.ns = "covariance_info";
        // text_marker.id = i * 2 + 1;
        // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        // text_marker.action = visualization_msgs::Marker::ADD;
        // text_marker.pose.position.x = voxel.voxel_center(0);
        // text_marker.pose.position.y = voxel.voxel_center(1);
        // text_marker.pose.position.z = voxel.voxel_center(2);
        // text_marker.scale.z = 0.15;
        // text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0;
        // text_marker.color.a = 0.8;
        
        // // Format voxel information
        // std::ostringstream display_text;
        // display_text << "Mean: [" << dist_mean(0) << ", " << dist_mean(1) << ", " << dist_mean(2) << "]"
        //              << "\nMax RCS: " << voxel.ref_rcs
        //              << "\nPoint Num: " << voxel.points.size()
        //              << "\nVoxel Center: [" << voxel.voxel_center(0) << ", " 
        //              << voxel.voxel_center(1) << ", " << voxel.voxel_center(2) << "]";
        
        // text_marker.text = display_text.str();
        // marker_array.markers.push_back(text_marker);
    }
    
    // Delete any previously existing markers that are no longer needed
    if (cur_voxel_num < last_voxel_num_) {
        for (int i = cur_voxel_num; i < last_voxel_num_; i++) {
            visualization_msgs::Marker cov_marker;
            cov_marker.header.frame_id = world_frame_id_;
            cov_marker.header.stamp = current_ros_time_;
            cov_marker.ns = "covariance";
            cov_marker.id = i * 2;
            cov_marker.type = visualization_msgs::Marker::SPHERE;
            cov_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(cov_marker);
            
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = world_frame_id_;
            text_marker.header.stamp = current_ros_time_;
            text_marker.ns = "covariance_info";
            text_marker.id = i * 2 + 1;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(text_marker);
        }
    }
    
    last_voxel_num_ = cur_voxel_num;
    
    return marker_array;
}

jsk_recognition_msgs::BoundingBoxArray Radar4VoxMapROS::getVoxelMapBoxArray() {
    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = world_frame_id_;
    bbox_array.header.stamp = current_ros_time_;
    
    double voxel_size = vox_map_config_.voxel_size;
    
    for (int i = 0; i < global_voxel_block_.size(); ++i) {
        const auto& voxel = global_voxel_block_[i];
        
        // Skip voxels that don't meet visualization criteria
        if (voxel.points.size() < vox_map_config_.voxel_visualize_min_point ||
            (!voxel.is_static && vox_map_config_.voxel_visualize_only_static)) {
            continue;
        }
        
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = world_frame_id_;
        bbox.header.stamp = current_ros_time_;
        bbox.label = voxel.is_static;
        bbox.dimensions.x = voxel_size;
        bbox.dimensions.y = voxel_size;
        bbox.dimensions.z = voxel_size;
        bbox.pose.position.x = voxel.voxel_center(0);
        bbox.pose.position.y = voxel.voxel_center(1);
        bbox.pose.position.z = voxel.voxel_center(2);
        bbox.pose.orientation.w = 1.0;
        bbox.value = 0.0;
        
        bbox_array.boxes.push_back(bbox);
    }
    
    return bbox_array;
}

visualization_msgs::MarkerArray Radar4VoxMapROS::getGraphMarkerArray() {
    visualization_msgs::MarkerArray marker_array;
    
    std::vector<SVisVertex> vertices = std::get<0>(graph_elements_);
    std::vector<SVisEdgeBinary> binary_edges = std::get<1>(graph_elements_);
    std::vector<SVisEdgeUnary> unary_edges = std::get<2>(graph_elements_);
    
    int vertex_num = vertices.size();
    int binary_edge_num = binary_edges.size();
    int unary_edge_num = unary_edges.size();
    
    ROS_INFO_STREAM("Graph: " << vertex_num << " vertices, " 
                  << binary_edge_num << " binary edges, " 
                  << unary_edge_num << " unary edges");
    
    // Setup pose array for graph vertices
    graph_node_pose_array_msg_.poses.clear();
    graph_node_pose_array_msg_.header.frame_id = world_frame_id_;
    graph_node_pose_array_msg_.header.stamp = current_ros_time_;
    
    for (const auto& vertex : vertices) {
        geometry_msgs::Pose vertex_pose;
        vertex_pose.position.x = vertex.pose(0, 3);
        vertex_pose.position.y = vertex.pose(1, 3);
        vertex_pose.position.z = vertex.pose(2, 3);
        
        Eigen::Matrix3d rotation_matrix = vertex.pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation_matrix);
        vertex_pose.orientation.x = quat.x();
        vertex_pose.orientation.y = quat.y();
        vertex_pose.orientation.z = quat.z();
        vertex_pose.orientation.w = quat.w();
        
        graph_node_pose_array_msg_.poses.push_back(vertex_pose);
    }
    
    // Create markers for binary edges
    int binary_edge_id = 0;
    for (const auto& edge : binary_edges) {
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = world_frame_id_;
        marker.header.stamp = current_ros_time_;
        marker.ns = "binary_edge_opt";
        marker.id = binary_edge_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Start and end points
        geometry_msgs::Point start_point;
        start_point.x = edge.vertex_pose_start(0, 3);
        start_point.y = edge.vertex_pose_start(1, 3);
        start_point.z = edge.vertex_pose_start(2, 3);
        
        geometry_msgs::Point end_point;
        end_point.x = edge.vertex_pose_end(0, 3);
        end_point.y = edge.vertex_pose_end(1, 3);
        end_point.z = edge.vertex_pose_end(2, 3);
        
        marker.points.push_back(start_point);
        marker.points.push_back(end_point);
        
        marker.scale.x = 0.1;  // Line width
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);
    }
    
    // Create markers for unary edges
    int unary_edge_id = 0;
    for (const auto& edge : unary_edges) {
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = world_frame_id_;
        marker.header.stamp = current_ros_time_;
        marker.ns = "unary_edge_opt";
        marker.id = unary_edge_id++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Vertex and measurement points
        geometry_msgs::Point vertex_point;
        vertex_point.x = edge.vertex_pose(0, 3);
        vertex_point.y = edge.vertex_pose(1, 3);
        vertex_point.z = edge.vertex_pose(2, 3);
        
        geometry_msgs::Point meas_point;
        meas_point.x = edge.measurement(0, 3);
        meas_point.y = edge.measurement(1, 3);
        meas_point.z = edge.measurement(2, 3);
        
        marker.points.push_back(vertex_point);
        marker.points.push_back(meas_point);
        
        marker.scale.x = 0.1;  // Line width
        marker.color.r = 0.0;  
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Blue
        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);
    }
    
    // Create markers for doppler-based unary edges
    int unary_edge_doppler_id = 0;
    for (const auto& edge : vertex_doppler_) {
        visualization_msgs::Marker marker;
        
        marker.header.frame_id = world_frame_id_;
        marker.header.stamp = current_ros_time_;
        marker.ns = "unary_edge_doppler";
        marker.id = unary_edge_doppler_id++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Vertex point and velocity vector
        geometry_msgs::Point vertex_point;
        vertex_point.x = edge.vertex_pose(0, 3);
        vertex_point.y = edge.vertex_pose(1, 3);
        vertex_point.z = edge.vertex_pose(2, 3);
        
        // Convert local velocity to global
        Eigen::Matrix3d rotation_matrix = edge.vertex_pose.block<3, 3>(0, 0);
        Eigen::Vector3d velocity_global = rotation_matrix * edge.velocity;
        
        geometry_msgs::Point doppler_end_point;
        doppler_end_point.x = vertex_point.x + velocity_global(0);
        doppler_end_point.y = vertex_point.y + velocity_global(1);
        doppler_end_point.z = vertex_point.z + velocity_global(2);
        
        marker.points.push_back(vertex_point);
        marker.points.push_back(doppler_end_point);
        
        marker.scale.x = 0.03;  // Shaft diameter
        marker.scale.y = 0.07;  // Head diameter
        marker.scale.z = 0.1;   // Head length
        
        marker.color.r = 1.0;   // Orange
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
        
        marker_array.markers.push_back(marker);
    }
    
    // Delete old markers that are no longer needed
    for (int i = binary_edge_id; i < last_published_binary_edge_num_; i++) {
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.ns = "binary_edge_opt";
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    
    for (int i = unary_edge_id; i < last_published_unary_edge_num_; i++) {
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.ns = "unary_edge_opt";
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    
    for (int i = unary_edge_doppler_id; i < last_published_unary_edge_doppler_num_; i++) {
        visualization_msgs::Marker marker;
        marker.id = i;
        marker.ns = "unary_edge_doppler";
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    
    // Update tracking variables
    last_published_binary_edge_num_ = binary_edge_id;
    last_published_unary_edge_num_ = unary_edge_id;
    last_published_unary_edge_doppler_num_ = unary_edge_doppler_id;
    
    return marker_array;
}

void Radar4VoxMapROS::sortEigenvaluesAndEigenvectors(Eigen::Vector3d& eigenvalues, Eigen::Matrix3d& eigenvectors) {
    // Pair eigenvalues with their corresponding eigenvectors
    std::vector<std::pair<double, Eigen::Vector3d>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
        eigen_pairs.emplace_back(eigenvalues(i), eigenvectors.col(i));
    }
    
    // Sort by eigenvalue in descending order
    std::sort(eigen_pairs.begin(), eigen_pairs.end(),
              [](const std::pair<double, Eigen::Vector3d>& a, 
                 const std::pair<double, Eigen::Vector3d>& b) { 
                  return a.first > b.first; 
              });
    
    // Reconstruct sorted eigenvalues and eigenvectors
    for (int i = 0; i < 3; ++i) {
        eigenvalues(i) = eigen_pairs[i].first;
        eigenvectors.col(i) = eigen_pairs[i].second;
    }
    
    // Ensure proper rotation matrix (determinant = 1)
    if (eigenvectors.determinant() < 0) {
        eigenvectors.col(0) = -eigenvectors.col(0);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "radar_4_vox_map_ros");
    ros::NodeHandle nh("~");
    
    ROS_INFO("Starting Radar4VoxMapROS node");
    
    Radar4VoxMapROS radar_4_vox_map_ros(nh);
    
    if (!radar_4_vox_map_ros.initialize()) {
        ROS_ERROR("Failed to initialize Radar4VoxMapROS");
        return 1;
    }
    
    ROS_INFO("Radar4VoxMapROS initialized, spinning...");
    ros::spin();
    
    return 0;
} 