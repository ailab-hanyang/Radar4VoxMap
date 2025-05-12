/*
 * @copyright Automotive Intelligence Lab, Hanyang University
 * @author Soyeong Kim <soyeongkim@hanyang.ac.kr> Jiwon Seok <pauljiwon96@gmail.com>
 * @file radar_point_cloud_type.hpp
 * @brief 
 * @version 1.0
 * @date 2024-08-16
 */

#ifndef RADAR_POINT_CLOUD_HPP
#define RADAR_POINT_CLOUD_HPP

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

// Ours
struct RadarPoint {
    float x;
    float y;
    float z;
    float range;
    float azi;
    float ele;
    float vel;
    float rcs;
    float power;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, range, range)
                                  (float, azi,   azi)
                                  (float, ele,   ele)
                                  (float, vel,   vel)
                                  (float, rcs,   rcs)
                                  (float, power,   power)
                                  )

typedef struct Point {
    double x;
    double y;
    double z;
} Point; // Temporary type

// datset
struct VodRadarPoint {
    float x;
    float y;
    float z;
    float v_r;
    float v_r_compensated;
    float RCS;
    float time;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(VodRadarPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, v_r, v_r)
                                  (float, v_r_compensated,   v_r_compensated)
                                  (float, RCS,   RCS)
                                  (float, time,   time)
                                  )

struct VodLiDARPoint {
    float x;
    float y;
    float z;
    float reflectance;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(VodLiDARPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, reflectance, reflectance)
                                  )

#endif // RADAR_POINT_CLOUD_HPP