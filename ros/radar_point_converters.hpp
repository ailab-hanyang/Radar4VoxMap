/**
 * @file radar_point_converters.hpp
 * @brief Utility functions for converting between radar point formats
 */

#ifndef __RADAR_POINT_CONVERTERS_HPP__
#define __RADAR_POINT_CONVERTERS_HPP__

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <sstream>

// Radar4VoxMap
#include "types/radar_point_cloud.hpp"

typedef std::vector<RadarPoint> RadarPointCloud;
typedef std::shared_ptr<RadarPointCloud> RadarPointCloudPtr;

// Field mapping structure definition
class CRadarFieldMapping {
public:
    CRadarFieldMapping() {}
    ~CRadarFieldMapping() {}

    typedef std::vector<std::pair<std::string, std::string>> RadarFieldMapping;
    typedef struct MappingOption {
        bool remap_rcs_from_pwr = false;
        bool remap_pwr_from_rcs = false;
        bool remap_azi_ele_from_xyz = false;
        bool remap_vel_from_vx_vy = false;
        bool remap_range_from_xyz = false;
    } MappingOption;

    /**
     * @brief View of Delft radar dataset format <new_field_name, original_field_name>
     *        RadarPoint: x, y, z, range, azi, ele, rcs, vel, power
     * @return RadarFieldMapping with fields set for View of Delft format
     */
    static RadarPointCloudPtr TransformFromViewOfDelft(const sensor_msgs::PointCloud2& input_cloud) {
        // Define field mapping
        RadarFieldMapping mapping;
        mapping.push_back(std::make_pair("x", "x"));
        mapping.push_back(std::make_pair("y", "y"));
        mapping.push_back(std::make_pair("z", "z"));
        mapping.push_back(std::make_pair("range", ""));
        mapping.push_back(std::make_pair("azi", ""));
        mapping.push_back(std::make_pair("ele", ""));
        mapping.push_back(std::make_pair("rcs", "RCS"));
        mapping.push_back(std::make_pair("vel", "v_r"));
        mapping.push_back(std::make_pair("power", ""));
        std::unordered_map<std::string, sensor_msgs::PointField> input_fields_map = CreateInputFieldsMap(mapping, input_cloud);

        // Remap point cloud fields based on the field mapping
        MappingOption mapping_option;
        mapping_option.remap_rcs_from_pwr = false;
        mapping_option.remap_pwr_from_rcs = true;
        mapping_option.remap_azi_ele_from_xyz = true;
        mapping_option.remap_vel_from_vx_vy = false;
        mapping_option.remap_range_from_xyz = true;
        RadarPointCloudPtr radar_point_cloud_ptr = RemapPointCloud2Fields(input_cloud, input_fields_map, mapping_option);

        return radar_point_cloud_ptr;
    }

    /**
     * @brief View of Delft radar dataset format <new_field_name, original_field_name>
     *        RadarPoint: x, y, z, range, azi, ele, rcs, vel, power
     * @return RadarFieldMapping with fields set for View of Delft format
     */
    static RadarPointCloudPtr TransformFromAfi910(const sensor_msgs::PointCloud2& input_cloud) {
        // Define field mapping
        RadarFieldMapping mapping;
        mapping.push_back(std::make_pair("x", "x"));
        mapping.push_back(std::make_pair("y", "y"));
        mapping.push_back(std::make_pair("z", "z"));
        mapping.push_back(std::make_pair("range", "range"));
        mapping.push_back(std::make_pair("azi", "azi_angle"));
        mapping.push_back(std::make_pair("ele", "ele_angle"));
        mapping.push_back(std::make_pair("rcs", ""));
        mapping.push_back(std::make_pair("vel", "vel"));
        mapping.push_back(std::make_pair("power", "power"));
        std::unordered_map<std::string, sensor_msgs::PointField> input_fields_map = CreateInputFieldsMap(mapping, input_cloud);

        // Remap point cloud fields based on the field mapping
        MappingOption mapping_option;
        mapping_option.remap_rcs_from_pwr = true;
        mapping_option.remap_pwr_from_rcs = false;
        mapping_option.remap_azi_ele_from_xyz = false;
        mapping_option.remap_vel_from_vx_vy = false;
        mapping_option.remap_range_from_xyz = false;
        RadarPointCloudPtr radar_point_cloud_ptr = RemapPointCloud2Fields(input_cloud, input_fields_map, mapping_option);

        return radar_point_cloud_ptr;
    }
    
    /**
     * @brief  mapping that Customn be defined by users
     * 
     * User need to fill in the fields that they want to use
     * 
     * @return RadarFieldMapping with empty fields for users to define
     */
    static RadarPointCloudPtr TransformFromArs548(const sensor_msgs::PointCloud2& input_cloud) {
        // Define field mapping
        RadarFieldMapping mapping;
        mapping.push_back(std::make_pair("x", "x"));
        mapping.push_back(std::make_pair("y", "y"));
        mapping.push_back(std::make_pair("z", "z"));
        mapping.push_back(std::make_pair("range", "r"));
        mapping.push_back(std::make_pair("azi", "azimuth"));
        mapping.push_back(std::make_pair("ele", ""));
        mapping.push_back(std::make_pair("rcs", "RCS"));
        mapping.push_back(std::make_pair("vel", "v"));
        mapping.push_back(std::make_pair("power", ""));
        std::unordered_map<std::string, sensor_msgs::PointField> input_fields_map = CreateInputFieldsMap(mapping, input_cloud);

        // Remap point cloud fields based on the field mapping
        MappingOption mapping_option;
        mapping_option.remap_rcs_from_pwr = false;
        mapping_option.remap_pwr_from_rcs = true;
        mapping_option.remap_azi_ele_from_xyz = true;
        mapping_option.remap_vel_from_vx_vy = false;
        mapping_option.remap_range_from_xyz = false;
        RadarPointCloudPtr radar_point_cloud_ptr = RemapPointCloud2Fields(input_cloud, input_fields_map, mapping_option);

        return radar_point_cloud_ptr;
    }

    /**
     * @brief  mapping that Customn be defined by users
     * 
     * User need to fill in the fields that they want to use
     * 
     * @return RadarFieldMapping with empty fields for users to define
     */
    static RadarPointCloudPtr Custom(const sensor_msgs::PointCloud2& input_cloud) {
        // Define field mapping
        RadarFieldMapping mapping;
        mapping.push_back(std::make_pair("x", "x"));
        mapping.push_back(std::make_pair("y", "y"));
        mapping.push_back(std::make_pair("z", "z"));
        mapping.push_back(std::make_pair("range", ""));
        mapping.push_back(std::make_pair("azi", ""));
        mapping.push_back(std::make_pair("ele", ""));
        mapping.push_back(std::make_pair("rcs", ""));
        mapping.push_back(std::make_pair("vel", ""));
        mapping.push_back(std::make_pair("power", ""));
        std::unordered_map<std::string, sensor_msgs::PointField> input_fields_map = CreateInputFieldsMap(mapping, input_cloud);

        // Remap point cloud fields based on the field mapping
        MappingOption mapping_option;
        mapping_option.remap_rcs_from_pwr = false;
        mapping_option.remap_pwr_from_rcs = true;
        mapping_option.remap_azi_ele_from_xyz = true;
        mapping_option.remap_vel_from_vx_vy = false;
        mapping_option.remap_range_from_xyz = false;
        RadarPointCloudPtr radar_point_cloud_ptr = RemapPointCloud2Fields(input_cloud, input_fields_map, mapping_option);

        return radar_point_cloud_ptr;
    }

    static std::unordered_map<std::string, sensor_msgs::PointField> CreateInputFieldsMap(const RadarFieldMapping& field_mapping, const sensor_msgs::PointCloud2& input_cloud) {
        std::unordered_map<std::string, sensor_msgs::PointField> input_fields_map;
        for (const auto& field : field_mapping) {
            // Skip empty fields
            const std::string& original_field_name = field.second;
            if (original_field_name == "") {
                continue;
            }

            // Find the original field name in the input cloud
            std::string target_field_name = field.first;
            for (const auto& input_field : input_cloud.fields) {
                if (original_field_name == input_field.name) {
                    input_fields_map[target_field_name] = input_field;
                    break;
                }
            }
        }
        return input_fields_map;
    }

    template <typename T>
    static T GetDataTypeData(const uint8_t* data_ptr, const sensor_msgs::PointField& input_field) {
        switch (input_field.datatype) {
            case sensor_msgs::PointField::INT8:
                return static_cast<T>(*reinterpret_cast<const int8_t*>(data_ptr));
            case sensor_msgs::PointField::UINT8:
                return static_cast<T>(*reinterpret_cast<const uint8_t*>(data_ptr));
            case sensor_msgs::PointField::INT16:
                return static_cast<T>(*reinterpret_cast<const int16_t*>(data_ptr));
            case sensor_msgs::PointField::UINT16:
                return static_cast<T>(*reinterpret_cast<const uint16_t*>(data_ptr));
            case sensor_msgs::PointField::INT32:
                return static_cast<T>(*reinterpret_cast<const int32_t*>(data_ptr));
            case sensor_msgs::PointField::UINT32:
                return static_cast<T>(*reinterpret_cast<const uint32_t*>(data_ptr));
            case sensor_msgs::PointField::FLOAT32:
                return static_cast<T>(*reinterpret_cast<const float*>(data_ptr));
            case sensor_msgs::PointField::FLOAT64:
                return static_cast<T>(*reinterpret_cast<const double*>(data_ptr));
            default:
                return static_cast<float>(0);  // 기본 반환 값 추가
        }
    }

    /**
     * @brief Remap point cloud fields based on a field mapping
     * @param input_cloud Input point cloud
     * @param field_mapping Field mapping configuration [new_field_name, original_point_field]
     * @return Remapped point cloud
     */
    static RadarPointCloudPtr RemapPointCloud2Fields(const sensor_msgs::PointCloud2& input_cloud,
                                            const std::unordered_map<std::string,sensor_msgs::PointField>& field_mapping,
                                            const MappingOption& mapping_option) 
    {
        RadarPointCloudPtr radar_point_cloud_ptr(new RadarPointCloud);
        radar_point_cloud_ptr->reserve(input_cloud.width * input_cloud.height);

        // Iterate over each point in the input cloud
        for (size_t point_idx = 0; point_idx < input_cloud.width * input_cloud.height; ++point_idx) {
            RadarPoint point;
            float v_x = 0.0f, v_y = 0.0f;
            bool has_vx_vy = false;
            bool has_range = false;

            // Extract data based on the field mapping and offsets
            for (const auto& kv : field_mapping) { // [new_field_name, original_point_field]
                const sensor_msgs::PointField& input_field = kv.second;
                const std::string& new_field_name = kv.first;

                // Calculate the offset of the point field in the input cloud
                size_t offset = point_idx * input_cloud.point_step + input_field.offset;

                // Get the pointer to the point field in the input cloud
                uint8_t* data_ptr = const_cast<uint8_t*>(&input_cloud.data[offset]);

                // Convert the point field to the new field name
                if (new_field_name == "x") {
                    point.x = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "y") {
                    point.y = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "z") {
                    point.z = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "range") {
                    point.range = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "azi") {
                    point.azi = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "ele") {
                    point.ele = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "vel") {
                    point.vel = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "rcs") {
                    point.rcs = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "power") {
                    point.power = GetDataTypeData<float>(data_ptr, input_field);
                } else if (new_field_name == "v_x") {
                    v_x = GetDataTypeData<float>(data_ptr, input_field);
                    has_vx_vy = true;
                } else if (new_field_name == "v_y") {
                    v_y = GetDataTypeData<float>(data_ptr, input_field);
                    has_vx_vy = true;
                }
            }

            // Remap range, azimuth, elevation, velocity, RCS, power
            if (mapping_option.remap_range_from_xyz) {
                point.range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            }
            if (mapping_option.remap_azi_ele_from_xyz) {
                if (has_range == true) {
                    point.azi = std::atan2(point.y, point.x);
                    point.ele = std::asin(point.z / point.range);
                } else {
                    point.azi = std::atan2(point.y, point.x);
                    point.ele = std::asin(point.z / sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
                }
            }
            if (mapping_option.remap_vel_from_vx_vy) {
                point.vel = std::sqrt(v_x * v_x + v_y * v_y);
            }
            if (mapping_option.remap_rcs_from_pwr) {
                double range = 0.0;
                if (has_range == true) {
                    range = point.range;
                } else {
                    range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                }
                // RCS = K * power * (range^4) K is some constant
                // Convert power_dB to watts
                double power_dB = point.power;
                double power_linear = std::pow(10.0, power_dB / 10.0);
                // Calculate linear RCS based on power and range^4 but afi910 is fit to range^2
                double rcs_linear = power_linear * std::pow(range, 2.0);
                // Convert linear RCS to dB, handle non-positive values
                if (rcs_linear > std::numeric_limits<double>::epsilon()) {
                    // Calculate RCS in dB (10 * log10(RCS_linear))
                    point.rcs = 10.0 * std::log10(rcs_linear);
                } else {
                    // Assign a very small dB value or handle as needed for non-positive linear RCS
                    point.rcs = -std::numeric_limits<double>::infinity(); 
                }
            }
            if (mapping_option.remap_pwr_from_rcs) {
                double range = 0.0;
                if (has_range == true) {
                    range = point.range;
                } else {
                    range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                }
                // power = K * RCS / (range^4), K is some constant
                point.power = point.rcs / (range*range*range*range);
            }
            radar_point_cloud_ptr->push_back(point);
        }

        return radar_point_cloud_ptr;
    }

    /** 
     * @brief Get the size of a point field based on its datatype
     * @param datatype Datatype of the point field
     * @return Size of the point field
     */
    static int getPointFieldSize(int datatype) {
        switch (datatype) {
            case sensor_msgs::PointField::INT8:
            case sensor_msgs::PointField::UINT8:
                return 1;
            case sensor_msgs::PointField::INT16:
            case sensor_msgs::PointField::UINT16:
                return 2;
            case sensor_msgs::PointField::INT32:
            case sensor_msgs::PointField::UINT32:
            case sensor_msgs::PointField::FLOAT32:
                return 4;
            case sensor_msgs::PointField::FLOAT64:
                return 8;
            default:
                throw std::runtime_error("Unknown PointField datatype");
        }
    }

    /**
     * @brief Convert from vector of RadarPoint to ROS PointCloud2 with basic fields
     * @param radar_points Input vector of RadarPoint
     * @param header ROS header for the output message
     * @return ROS PointCloud2 message
     */
    static sensor_msgs::PointCloud2::Ptr toROSPointCloud2(const std::vector<RadarPoint>& radar_points, const std_msgs::Header& header) {
        pcl::PointCloud<RadarPoint> pcl_cloud;
        pcl_cloud.points.reserve(radar_points.size());

        for (const auto& pt : radar_points) {
            pcl_cloud.points.push_back(pt);
        }

        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(pcl_cloud, *cloud_msg);
        cloud_msg->header = header;
 
        return cloud_msg;
    }
};
#endif // __RADAR_POINT_CONVERTERS_HPP__ 