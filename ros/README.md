# Radar4VoxMap ROS Wrapper

## Overview

`radar_4_vox_map_ros` is a real-time ROS wrapper for the `radar_4_vox_map` algorithm. This node processes radar point clouds in real-time to build voxel maps and perform localization.

## Usage

### Running the Node

You can run the node with the following command:

```bash
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch
```

### Launch File Parameters

The following parameters can be used to adjust the node's behavior:

- `input_radar_topic` (default: `/radar/points`): Input radar point cloud topic
- `radar_dataset_type` (default: `auto`): Radar dataset type (auto, ti, ouster, navtech, custom)
- `algorithm_ini_path` (default: `config/radar_4_vox_map.ini`): Algorithm configuration file path
- `world_frame_id` (default: `world`): World coordinate frame ID
- `radar_frame_id` (default: `radar`): Radar coordinate frame ID
- `base_link_frame_id` (default: `base_link`): Base link coordinate frame ID
- `publish_tf` (default: `true`): Whether to publish TF messages
- `publish_voxel_map` (default: `true`): Whether to publish voxel map visualization
- `publish_graph` (default: `true`): Whether to publish graph optimization visualization
- `visualize` (default: `true`): Whether to run RViz visualization

Example:
```bash
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch input_radar_topic:=/my_radar/points radar_dataset_type:=ti
```

## Radar Point Cloud Mapping Tutorial

The system supports various radar point cloud formats through a flexible field mapping system. Understanding how to configure the field mapping is essential for working with different radar datasets.

### Field Mapping Overview

The `RadarFieldMapping` structure defines how field names in your radar's PointCloud2 message are mapped to the internal data structures. This allows processing of data from different radar sensors that might use different field naming conventions.

```cpp
struct RadarFieldMapping {
    std::string x_field = "x";                  // X coordinate field name
    std::string y_field = "y";                  // Y coordinate field name
    std::string z_field = "z";                  // Z coordinate field name
    std::string intensity_field = "intensity";  // Intensity/RCS field name
    std::string doppler_field = "";             // Doppler velocity field name (empty if not used)
    std::string azimuth_field = "";             // Azimuth angle field name (empty if not used)
    std::string elevation_field = "";           // Elevation angle field name (empty if not used)
    std::string range_field = "";               // Range field name (empty if not used)
    std::string snr_field = "";                 // SNR field name (empty if not used)
    std::string is_static_field = "";           // Static object flag field name (empty if not used)
    std::string id_field = "";                  // ID field name (empty if not used)
};
```

### Predefined Dataset Types

The system provides several predefined mappings for common radar formats:

1. **Auto** (default): Automatically detects fields based on names.
2. **TI Radar**: Optimized for Texas Instruments radar formats.
3. **Ouster LiDAR**: Mapped for Ouster LiDAR data.
4. **Navtech Radar**: Configured for Navtech radar systems.
5. **Custom**: User-defined mapping that can be specified in the code.

### Using Different Dataset Types

You can select a dataset type via the launch file parameter:

```bash
# Use auto detection
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch

# Use TI radar format
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch radar_dataset_type:=ti

# Use Ouster LiDAR format
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch radar_dataset_type:=ouster

# Use Navtech radar format
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch radar_dataset_type:=navtech

# Use custom field mapping
roslaunch radar_4_vox_map radar_4_vox_map_ros.launch radar_dataset_type:=custom
```

### Creating a Custom Field Mapping

To work with a unique radar dataset format, you can define a custom mapping:

1. **Modify the setupFieldMapping() method** in the `radar_4_vox_map_ros.cpp` file:

```cpp
else if (radar_dataset_type_ == "custom") {
    // Define custom field mapping for your radar format
    field_mapping_.x_field = "x";
    field_mapping_.y_field = "y";
    field_mapping_.z_field = "z";
    field_mapping_.intensity_field = "intensity"; 
    field_mapping_.doppler_field = "velocity";       // Doppler velocity field
    field_mapping_.azimuth_field = "angle_h";        // Horizontal angle
    field_mapping_.elevation_field = "angle_v";      // Vertical angle
    field_mapping_.range_field = "distance";         // Range measurement
    field_mapping_.snr_field = "snr";                // Signal-to-noise ratio
    field_mapping_.is_static_field = "is_stationary"; // Static point indicator
    field_mapping_.id_field = "target_id";           // Point ID
    ROS_INFO("Using custom field mapping");
}
```

2. **Add a new radar type** by defining a static method in the `radar_point_converters.hpp` file:

```cpp
static RadarFieldMapping YourRadarType() {
    RadarFieldMapping mapping;
    mapping.x_field = "x";
    mapping.y_field = "y";
    mapping.z_field = "z";
    mapping.intensity_field = "intensity";
    mapping.doppler_field = "velocity";
    // Set other fields as needed
    return mapping;
}
```

3. **Update the setupFieldMapping() method** to include your new type:

```cpp
else if (radar_dataset_type_ == "your_type") {
    field_mapping_ = RadarFieldMapping::YourRadarType();
    ROS_INFO("Using Your Radar type field mapping");
}
```

### Analyzing Available Fields

When processing radar data with automatic field detection, the system logs the available fields in the first received message:

```
Detected fields for this dataset:
X: x, Y: y, Z: z
Intensity: intensity, Doppler: velocity
Azimuth: angle_h, Elevation: angle_v, Range: distance
SNR: snr, Is_Static: is_stationary, ID: target_id
```

You can use this information to:
1. Understand what fields are available in your dataset
2. Create a custom mapping based on the detected fields
3. Verify that your mapping correctly matches the data structure

### Advanced Usage: Direct Function Calls

For direct access to conversion functions in your C++ code:

```cpp
// Using automatic field detection
std::vector<SRadarPoint> radar_points = fromROSPointCloud2(radar_msg);

// Using specific field mapping
RadarFieldMapping mapping = RadarFieldMapping::TIRadar();
std::vector<SRadarPoint> radar_points = fromROSPointCloud2WithMapping(radar_msg, mapping);

// Custom field mapping
RadarFieldMapping mapping;
mapping.x_field = "x";
mapping.y_field = "y";
mapping.z_field = "z";
mapping.doppler_field = "velocity";
std::vector<SRadarPoint> radar_points = fromROSPointCloud2WithMapping(radar_msg, mapping);

// Get field information
std::string field_info = getPointCloudFieldInfo(radar_msg);
RadarFieldMapping template_mapping = createFieldMappingTemplate(radar_msg);
```

## Published Topics

The node publishes the following topics:

- `/radar_4_vox_map/radar_local_map` (sensor_msgs/PointCloud2): Local map point cloud
- `/radar_4_vox_map/radar_points` (sensor_msgs/PointCloud2): Radar points in global frame
- `/radar_4_vox_map/estimated_pose_array` (geometry_msgs/PoseArray): Estimated pose array
- `/radar_4_vox_map/estimated_pose` (geometry_msgs/PoseStamped): Latest estimated pose
- `/radar_4_vox_map/voxel_map_marker_array` (visualization_msgs/MarkerArray): Voxel map visualization markers
- `/radar_4_vox_map/voxel_map_voxel_box_array` (jsk_recognition_msgs/BoundingBoxArray): Voxel map bounding boxes
- `/radar_4_vox_map/graph_marker_array` (visualization_msgs/MarkerArray): Graph optimization visualization markers
- `/radar_4_vox_map/graph_vertex_pose_array` (geometry_msgs/PoseArray): Graph vertex pose array
- `/tf` (tf2_msgs/TFMessage): TF transform messages

## Configuration File

Algorithm settings are configured through the INI file specified by the `algorithm_ini_path` parameter. This file allows you to adjust:

- Multithreading settings
- Sensor noise model settings
- Optimization settings
- ICP algorithm settings
- Voxel map settings
- Dynamic object filtering settings
- Visualization settings

Refer to the sample INI file for detailed configuration options 