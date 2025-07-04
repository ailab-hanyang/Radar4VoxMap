/*
 * @copyright Automotive Intelligence Lab, Hanyang University
 * @author wodud3743@gmail.com
 * @file ros_tools.hpp
 * @brief converting algorithm struct to ros data struct
 * @version 1.0
 * @date 2024-09-02
 */

#ifndef __ROS_TOOLS_HPP__
#define __ROS_TOOLS_HPP__
#pragma once


#include <vector>
#include <deque>
#include <string>
#include <map>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/point_cloud2_iterator.h"
#include <regex>

#include "utils.hpp"
#include "data_types.hpp"

inline std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

inline std::unique_ptr<sensor_msgs::PointCloud2> CreatePointCloud2Msg(const size_t n_points,
                                                         const std_msgs::Header &header,
                                                         bool timestamp = false) {
    auto cloud_msg = std::make_unique<sensor_msgs::PointCloud2>();
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    cloud_msg->header = header;
    cloud_msg->header.frame_id = FixFrameId(cloud_msg->header.frame_id);
    cloud_msg->fields.clear();
    int offset = 0;
    offset = addPointField(*cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);

    offset = addPointField(*cloud_msg, "range", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "azi", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "ele", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "vel", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "rcs", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(*cloud_msg, "power", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
    if (timestamp) {
        // assuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
        offset = addPointField(*cloud_msg, "time", 1, sensor_msgs::PointField::FLOAT64, offset);
        offset += sizeOfPointField(sensor_msgs::PointField::FLOAT64);
    }

    // Resize the point cloud accordingly
    cloud_msg->point_step = offset;
    cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step;
    cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

inline void FillRadarPointCloud2XYZ(const std::vector<RadarPoint> &points, sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");

    sensor_msgs::PointCloud2Iterator<float> msg_range(msg, "range");
    sensor_msgs::PointCloud2Iterator<float> msg_azi(msg, "azi");
    sensor_msgs::PointCloud2Iterator<float> msg_ele(msg, "ele");
    sensor_msgs::PointCloud2Iterator<float> msg_vel(msg, "vel");
    sensor_msgs::PointCloud2Iterator<float> msg_rcs(msg, "rcs");
    sensor_msgs::PointCloud2Iterator<float> msg_power(msg, "power");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z, 
            ++msg_range, ++msg_azi, ++msg_ele, ++msg_vel, ++msg_rcs, ++msg_power) {
        const RadarPoint &point = points[i];
        *msg_x = point.x;
        *msg_y = point.y;
        *msg_z = point.z;

        *msg_range = point.range;
        *msg_azi = point.azi;
        *msg_ele = point.ele;
        *msg_vel = point.vel;
        *msg_rcs = point.rcs;
        *msg_power = point.power;
    }
}

inline std::unique_ptr<sensor_msgs::PointCloud2> RadarPointToPointCloud2(const std::vector<RadarPoint> &points,
                                                       const std_msgs::Header &header) {
    auto msg = CreatePointCloud2Msg(points.size(), header);
    FillRadarPointCloud2XYZ(points, *msg);
    return msg;
}


#endif // __ROS_TOOLS_HPP__
