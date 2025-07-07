#include "voxeled_rcs_mapper/voxeled_rcs_mapper_2d.hpp"

namespace{
struct ResultTuple {
    ResultTuple() = default;
    ResultTuple(std::size_t n) {
        vec_cor_origin_pair.reserve(n);
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    std::vector<SRadarPoint> source;
    std::vector<SRadarPoint> target;
};
}

void Voxel2DHashMap::Init(VoxelRcsMapperConfig config){
    std::cout<<"[VoxelHashMap] Init"<<std::endl;
    m_config = config;
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>>  Voxel2DHashMap::GetCorrespondencesWithSet(
    const RadarPointVector &points, double max_correspondance_distance) const {
 
    auto GetClosestNeighboor = [&](const SRadarPoint &point) {

        std::vector<Voxel> voxels = GetSearchVoxel(point);

        using RadarPointVector = std::vector<SRadarPoint>;
        RadarPointVector neighboors;
        neighboors.reserve(voxels.size() * m_config.voxel_max_point_num);
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                const auto &points = search->second.points;
                if (!points.empty()) {
                    for (const auto &point : points) {
                        neighboors.emplace_back(point);
                    }
                }
            }
        }

        // Find the closest neighbor
        SRadarPoint closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighbor : neighboors) {
            double distance = sqrt(pow(neighbor.pose(0) - point.pose(0), 2) + pow(neighbor.pose(1) - point.pose(1), 2));
            if (distance < closest_distance2) {
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        }

        return closest_neighbor;
    };

    // Limit the maximum search distance
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    RadarPointVector source, target;
    source.reserve(points.size());
    target.reserve(points.size());
    vec_cor_origin_pair.reserve(points.size());

    for (int i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        SRadarPoint closest_neighboors = GetClosestNeighboor(point);
        double distance = sqrt(pow(closest_neighboors.pose(0) - point.pose(0), 2) + pow(closest_neighboors.pose(1) - point.pose(1), 2));

        if (distance < max_correspondance_distance) {
            source.emplace_back(point);
            target.emplace_back(closest_neighboors);
            vec_cor_origin_pair.emplace_back(std::make_pair(static_cast<int>(source.size()) - 1, i));
        }
    }

    return std::make_tuple(vec_cor_origin_pair, source, target);

}

std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>>  
Voxel2DHashMap::GetCorrespondencesWithSetTbb(
    const RadarPointVector &points, double max_correspondance_distance) const {

    auto GetClosestNeighboor = [&](const SRadarPoint &point) {
        std::vector<Voxel> voxels = GetSearchVoxel(point);

        RadarPointVector neighboors;
        neighboors.reserve(voxels.size() * m_config.voxel_max_point_num);
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                const auto &points = search->second.points;
                if (!points.empty()) {
                    neighboors.insert(neighboors.end(), points.begin(), points.end());
                }
            }
        }

        SRadarPoint closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighbor : neighboors) {
            double distance = sqrt(pow(neighbor.pose(0) - point.pose(0), 2) + pow(neighbor.pose(1) - point.pose(1), 2));
            if (distance < closest_distance2) {
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        }

        return closest_neighbor;
    };

    using points_iterator = RadarPointVector::const_iterator;
    
    auto result = tbb::parallel_reduce(
        tbb::blocked_range<points_iterator>(points.cbegin(), points.cend()),
        // Initial value: ResultTuple creation
        ResultTuple(0),
        // 1st lambda: Parallel calculation
        [&](const tbb::blocked_range<points_iterator>& r, ResultTuple res) -> ResultTuple {
            auto &[vec_cor_origin_pair, source, target] = res;
            
            for (auto it = r.begin(); it != r.end(); ++it) {
                const auto &point = *it;
                SRadarPoint closest_neighbor = GetClosestNeighboor(point);
                double distance = sqrt(pow(closest_neighbor.pose(0) - point.pose(0), 2) + pow(closest_neighbor.pose(1) - point.pose(1), 2));
                if (distance < max_correspondance_distance) {
                    int source_index = static_cast<int>(source.size());
                    source.emplace_back(point);
                    target.emplace_back(closest_neighbor);
                    vec_cor_origin_pair.emplace_back(source_index, static_cast<int>(it - points.begin()));
                }
            }

            return res;
        },
        // 2nd lambda: Parallel combination
        [](ResultTuple a, const ResultTuple& b) -> ResultTuple {
            auto &[vec_a, src_a, tgt_a] = a;
            const auto &[vec_b, src_b, tgt_b] = b;

            int offset = static_cast<int>(src_a.size());
            
            vec_a.reserve(vec_a.size() + vec_b.size());
            for (const auto &p : vec_b) {
                vec_a.emplace_back(p.first + offset, p.second);
            }

            src_a.insert(src_a.end(), std::make_move_iterator(src_b.begin()), std::make_move_iterator(src_b.end()));
            tgt_a.insert(tgt_a.end(), std::make_move_iterator(tgt_b.begin()), std::make_move_iterator(tgt_b.end()));

            return a;
        }
    );

    // Return the result of parallel calculation
    auto &[vec, source, target] = result;
    return std::make_tuple(vec, source, target);
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> Voxel2DHashMap::GetCorrespondencesVoxelCovWithSet(
    const RadarPointVector &points, double max_correspondance_distance) const {

    auto GetClosestCov = [&](const SRadarPoint &point) {
        std::vector<Voxel> voxels = GetSearchVoxel(point);
        
        using VoxelBlockVector = std::vector<Voxel2DHashMap::VoxelBlock>;
        VoxelBlockVector neighboors_voxel_block;
        neighboors_voxel_block.reserve(voxels.size());
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                neighboors_voxel_block.emplace_back(search->second);
            }
        }

        if (neighboors_voxel_block.empty()) {
            // Return the default value or perform other processing
            return SRadarPoint(); // Use the default constructor to return the default value
        }

        // Find the closest voxel
        Voxel2DHashMap::VoxelBlock closest_voxel_block = neighboors_voxel_block[0]; // Initialize
        // VoxelHashMap::VoxelBlock closest_voxel_block;
        SRadarPoint closest_cov_point;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighboor_voxel_block : neighboors_voxel_block) {
            double distance = sqrt(pow(neighboor_voxel_block.covariance.mean(0) - point.pose(0), 2) + pow(neighboor_voxel_block.covariance.mean(1) - point.pose(1), 2));
            if (distance < closest_distance2) {
                closest_voxel_block = neighboor_voxel_block;
                closest_distance2 = distance;
            }
        }

        // TODO: Need to check if there is additional information 08/29
        closest_cov_point.pose.head<2>() = closest_voxel_block.covariance.mean;
        closest_cov_point.cov.setZero();
        closest_cov_point.cov.block<2, 2>(0, 0) = closest_voxel_block.covariance.cov;
        closest_cov_point.rcs = closest_voxel_block.ref_rcs;
        closest_cov_point.is_static = closest_voxel_block.is_static;
        closest_cov_point.power = closest_voxel_block.points.size(); // FIXME: 임시로 power에 cov point 개수 넣어놓음
        
        return closest_cov_point;
    };

    // Limit the maximum search distance and find the closest point
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    RadarPointVector source, target_cov;
    source.reserve(points.size());
    target_cov.reserve(points.size());
    vec_cor_origin_pair.reserve(points.size());

    for (int i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        SRadarPoint closest_neighbor = GetClosestCov(point);

        if(closest_neighbor.pose.isApprox(Eigen::Vector3d())) continue;

        double distance = sqrt(pow(closest_neighbor.pose(0) - point.pose(0), 2) + pow(closest_neighbor.pose(1) - point.pose(1), 2));
        if (distance < max_correspondance_distance) {
            source.emplace_back(point);
            target_cov.emplace_back(closest_neighbor);
            vec_cor_origin_pair.emplace_back(std::make_pair(static_cast<int>(source.size()) - 1, i));
        }
    }

    return std::make_tuple(vec_cor_origin_pair, source, target_cov);
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
Voxel2DHashMap::GetCorrespondencesVoxelCovWithSetTbb(const RadarPointVector &points, double max_correspondance_distance) const {

    ResultTuple result(points.size());

    // Find the closest voxel function
    auto GetClosestCov = [&](const SRadarPoint &point) {
        std::vector<Voxel> voxels = GetSearchVoxel(point);

        using VoxelBlockVector = std::vector<Voxel2DHashMap::VoxelBlock>;
        VoxelBlockVector neighboors_voxel_block;
        neighboors_voxel_block.reserve(voxels.size());
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                neighboors_voxel_block.emplace_back(search->second);
            }
        }

        if (neighboors_voxel_block.empty()) {
            return SRadarPoint(); // Use the default constructor to return the default value
        }

        // Find the closest voxel
        Voxel2DHashMap::VoxelBlock closest_voxel_block = neighboors_voxel_block[0];
        SRadarPoint closest_cov_point;
        double closest_distance2 = std::numeric_limits<double>::max();
        for (const auto &neighboor_voxel_block : neighboors_voxel_block) {
            double distance = sqrt(pow(neighboor_voxel_block.covariance.mean(0) - point.pose(0), 2) + pow(neighboor_voxel_block.covariance.mean(1) - point.pose(1), 2));
            if (distance < closest_distance2) {
                closest_voxel_block = neighboor_voxel_block;
                closest_distance2 = distance;
            }
        }

        closest_cov_point.pose.head<2>() = closest_voxel_block.covariance.mean;
        closest_cov_point.cov.setZero();
        closest_cov_point.cov.block<2, 2>(0, 0) = closest_voxel_block.covariance.cov;
        closest_cov_point.rcs = closest_voxel_block.ref_rcs;
        closest_cov_point.is_static = closest_voxel_block.is_static;
        closest_cov_point.power = closest_voxel_block.points.size(); // FIXME: Temporarily put the number of cov points in power

        return closest_cov_point;
    };

    using points_iterator = RadarPointVector::const_iterator;

    result = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<points_iterator>(points.cbegin(), points.cend()),
        // Identity
        result,
        // 1st lambda: Parallel computation
        [&](const tbb::blocked_range<points_iterator>& r, ResultTuple res) -> ResultTuple {
            // auto &[vec_cor_origin_pair, source, target_cov] = res;
            res.source.reserve(r.size());
            res.target.reserve(r.size());
            res.vec_cor_origin_pair.reserve(r.size());

            for (auto it = r.begin(); it != r.end(); ++it) {
                const auto &point = *it;
                SRadarPoint closest_neighbor = GetClosestCov(point);

                if(closest_neighbor.pose.isApprox(Eigen::Vector3d())) continue;

                double distance = sqrt(pow(closest_neighbor.pose(0) - point.pose(0), 2) + pow(closest_neighbor.pose(1) - point.pose(1), 2));
                if (distance < max_correspondance_distance) {
                    int source_index = static_cast<int>(res.source.size());
                    res.source.emplace_back(point);
                    res.target.emplace_back(closest_neighbor);
                    res.vec_cor_origin_pair.emplace_back(source_index, static_cast<int>(it - points.begin()));
                }
            }

            return res;
        },
        // 2nd lambda: Parallel reduction
        [](ResultTuple a, const ResultTuple& b) -> ResultTuple {
            auto &[vec_a, src_a, tgt_a] = a;
            const auto &[vec_b, src_b, tgt_b] = b;

            int offset = static_cast<int>(src_a.size());

            vec_a.reserve(vec_a.size() + vec_b.size());
            for (const auto &p : vec_b) {
                vec_a.emplace_back(p.first + offset, p.second);
            }

            src_a.insert(src_a.end(), std::make_move_iterator(src_b.begin()), std::make_move_iterator(src_b.end()));
            tgt_a.insert(tgt_a.end(), std::make_move_iterator(tgt_b.begin()), std::make_move_iterator(tgt_b.end()));

            return a;
        }
    );

    return std::make_tuple(result.vec_cor_origin_pair, result.source, result.target);;
}

// Return all voxels associated with the point
std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>>  Voxel2DHashMap::GetCorrespondencesAllVoxelCovWithSet(
    const RadarPointVector &points, double max_correspondance_distance) const {

    auto GetAllCovPoint = [&](const SRadarPoint &point) {
        std::vector<Voxel> voxels = GetSearchVoxel(point);

        using RadarPointVector = std::vector<SRadarPoint>;
        RadarPointVector neighboors;
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                SRadarPoint cov_point;
                // TODO: Need to check if there is additional information 08/29
                cov_point.pose.head<2>() = search->second.covariance.mean;
                cov_point.cov.setZero();
                cov_point.cov.block<2, 2>(0, 0) = search->second.covariance.cov;
                cov_point.rcs = search->second.ref_rcs;
                cov_point.is_static = search->second.is_static;
                cov_point.power = search->second.points.size(); // FIXME: temporarily put the number of cov points in power

                neighboors.push_back(cov_point);
            }
        }

        return neighboors;
    };

    // Limit the maximum search distance
    std::vector<std::pair<int, int>> vec_cor_origin_pair;
    RadarPointVector source, target_cov;
    source.reserve(points.size());
    target_cov.reserve(points.size());
    vec_cor_origin_pair.reserve(points.size());

    for (int i = 0; i < points.size(); ++i){
        const auto &point = points[i];
        std::vector<SRadarPoint> closest_neighboors = GetAllCovPoint(point);
        for(const auto & neighboor: closest_neighboors){
            double distance = sqrt(pow(neighboor.pose(0) - point.pose(0), 2) + pow(neighboor.pose(1) - point.pose(1), 2));
            if (distance < max_correspondance_distance) {
                source.emplace_back(point);
                target_cov.emplace_back(neighboor);
                vec_cor_origin_pair.emplace_back(std::make_pair(static_cast<int>(source.size()) - 1, i));
            }
        }
    }

    return std::make_tuple(vec_cor_origin_pair, source, target_cov);
}

std::tuple<std::vector<std::pair<int, int>>, std::vector<SRadarPoint>, std::vector<SRadarPoint>> 
Voxel2DHashMap::GetCorrespondencesAllVoxelCovWithSetTbb(const RadarPointVector &points, double max_correspondance_distance) const {

    // Function to get all points associated with all voxels for a given point
    auto GetAllCovPoint = [&](const SRadarPoint &point) {
        std::vector<Voxel> voxels = GetSearchVoxel(point);

        using RadarPointVector = std::vector<SRadarPoint>;
        RadarPointVector neighboors;
        for (const auto &voxel : voxels) {
            auto search = m_voxel_map.find(voxel);
            if (search != m_voxel_map.end()) {
                SRadarPoint cov_point;
                // TODO: Need to check if there is additional information 08/29
                cov_point.pose.head<2>() = search->second.covariance.mean;
                cov_point.cov.setZero();
                cov_point.cov.block<2, 2>(0, 0) = search->second.covariance.cov;
                cov_point.rcs = search->second.ref_rcs;
                cov_point.is_static = search->second.is_static;
                cov_point.power = search->second.points.size(); // FIXME: Temporarily put the number of cov points in power

                neighboors.push_back(cov_point);
            }
        }

        return neighboors;
    };

    using points_iterator = RadarPointVector::const_iterator;

    auto result = tbb::parallel_reduce(
        tbb::blocked_range<points_iterator>(points.cbegin(), points.cend()),
        ResultTuple(0),
        [&](const tbb::blocked_range<points_iterator>& r, ResultTuple res) -> ResultTuple {
            auto &[vec_cor_origin_pair, source, target_cov] = res;

            for (auto it = r.begin(); it != r.end(); ++it) {
                const auto &point = *it;
                std::vector<SRadarPoint> closest_neighboors = GetAllCovPoint(point);
                
                for (const auto &neighboor : closest_neighboors) {
                    double distance = sqrt(pow(neighboor.pose(0) - point.pose(0), 2) + pow(neighboor.pose(1) - point.pose(1), 2));
                    if (distance < max_correspondance_distance) {
                        int source_index = static_cast<int>(source.size());
                        source.emplace_back(point);
                        target_cov.emplace_back(neighboor);
                        vec_cor_origin_pair.emplace_back(source_index, static_cast<int>(it - points.begin()));
                    }
                }
            }

            return res;
        },
        [](ResultTuple a, const ResultTuple& b) -> ResultTuple {
            auto &[vec_a, src_a, tgt_a] = a;
            const auto &[vec_b, src_b, tgt_b] = b;

            int offset = static_cast<int>(src_a.size());

            vec_a.reserve(vec_a.size() + vec_b.size());
            for (const auto &p : vec_b) {
                vec_a.emplace_back(p.first + offset, p.second);
            }

            src_a.insert(src_a.end(), std::make_move_iterator(src_b.begin()), std::make_move_iterator(src_b.end()));
            tgt_a.insert(tgt_a.end(), std::make_move_iterator(tgt_b.begin()), std::make_move_iterator(tgt_b.end()));

            return a;
        }
    );

    auto &[vec, source, target_cov] = result;
    return std::make_tuple(vec, source, target_cov);
}

std::vector<SRadarPoint> Voxel2DHashMap::Pointcloud() const {
    std::vector<SRadarPoint> points;
    // Allocate the maximum number of points in a voxel * the total number of voxels
    points.reserve(m_config.voxel_max_point_num * m_voxel_map.size());

    std::for_each(m_voxel_map.cbegin(), m_voxel_map.cend(), [&](const auto &map_element) {
        const auto &voxel_points = map_element.second.points;
        points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    });
    points.shrink_to_fit();

    return points;
}

std::vector<SRadarPoint> Voxel2DHashMap::StaticPointcloud() const {
    std::vector<SRadarPoint> points;
    // Allocate the maximum number of points in a voxel * the total number of voxels
    points.reserve(m_config.voxel_max_point_num * m_voxel_map.size());

    std::for_each(m_voxel_map.cbegin(), m_voxel_map.cend(), [&](const auto &map_element) {
        const auto &voxel_points = map_element.second.points;
        std::for_each(voxel_points.cbegin(), voxel_points.cend(), [&](const auto &point) {
            if (point.is_static && point.range < 100) {
                points.push_back(point);
            }
        });
    });
    points.shrink_to_fit();

    return points;
}

std::vector<Voxel2DHashMap::VoxelBlock> Voxel2DHashMap::GetAllVoxel() const {
    std::vector<VoxelBlock> o_vec_vox_block;
    // Reserve the maximum number of voxels
    o_vec_vox_block.reserve(m_voxel_map.size());
    for (auto &pair : m_voxel_map) {
        if(pair.second.points.size() > 0)
            o_vec_vox_block.push_back(pair.second);
    }
    return o_vec_vox_block;
}


// Update the point that has moved globally
void Voxel2DHashMap::Update(const RadarPointVector &points, const Eigen::Vector2d &origin) {
    AddPoints(points);
    // RemovePointsFarFromLocation(origin); // TODO: Will be managed by the graph
    // RemovePointsFarfromTime(points.back().timestamp);
}

void Voxel2DHashMap::Update(const RadarPointVector &points, const Eigen::Isometry2d &pose) {
    int n = points.size();
    RadarPointVector points_transformed(n);
    Eigen::Matrix2d rotation = pose.rotation();
    Eigen::Vector2d translation = pose.translation();
        
    for (int i = 0 ; i < n ; ++i){
        points_transformed[i] = points[i];
        Eigen::Vector2d local_point = {points[i].pose[0], points[i].pose[1]};
        Eigen::Vector2d global_point = rotation * local_point + translation;
        points_transformed[i].pose = {global_point[0], global_point[1], 0.0};
    }

    Update(points_transformed, translation);
}

void Voxel2DHashMap::UpdateGlobal(const RadarPointVector &points, const Eigen::Isometry2d &pose) {
    // RadarPointVector points_transformed(points.size());
    Update(points, pose);
}


void Voxel2DHashMap::AddPoints(const RadarPointVector &points) {
    if(points.size() == 0)
        return;

    Voxel2DHashMap::Voxel voxel;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) { // point = SRadarPoint
        // const auto voxel = PointToVoxel(point.pose,  m_config.voxel_size);
        // Convert point to voxel position {x, y}
        const Eigen::Vector2d p2d = {point.pose(0), point.pose(1)};
        PointToVoxel(p2d, voxel, m_config.voxel_size);


        auto search = m_voxel_map.find(voxel);
        if (search != m_voxel_map.end()) { // If the voxel exists, check if there is enough space in the voxel and add it
            auto &voxel_block = search->second;
            voxel_block.AddPoint(point);
            voxel_block.is_changed = true;
        } else { // If the voxel does not exist, add a new voxel
            Eigen::Vector2d voxel_center = voxel.template cast<double>() * m_config.voxel_size + 
                        Eigen::Vector2d(m_config.voxel_size / 2, m_config.voxel_size / 2); 

            // 2-D 공분산 객체 생성 후 필드 직접 대입
            SCovariance2D cov2d;
            cov2d.cov  = Eigen::Matrix2d::Identity();
            cov2d.mean = p2d;

            VoxelBlock voxel_block{{point},
                                    cov2d,
                                    voxel_center,
                                    VoxelToSizeT(voxel),
                                    0.0f, false, m_config.voxel_max_point_num};
            voxel_block.points.reserve(m_config.voxel_max_point_num); // Allocate memory for the maximum number of points in the voxel
            m_voxel_map.insert({voxel, std::move(voxel_block)});
        }
        
    });

}

void Voxel2DHashMap::RemovePointsFarFromLocation(const Eigen::Vector2d &origin) {
    for (auto it = m_voxel_map.begin(); it != m_voxel_map.end();) {
        const auto &pt = it->second.points.front().pose.head<2>(); // second = VoxelBlock. Search the first point of the voxel
        const auto max_distance2 = m_config.voxel_map_max_distance * m_config.voxel_map_max_distance;
        if ((pt - origin).squaredNorm() > (max_distance2)) {
            it = m_voxel_map.erase(it);
        } else {
            ++it;
        }
    }
}

// Remove all points in the voxel map that have the same frame_idx
void Voxel2DHashMap::RemovePointsFrameIdx(const int idx) {
    for (auto it = m_voxel_map.begin(); it != m_voxel_map.end();) {
        auto& points = it->second.points; // Reference to the point vector of the current voxel

        // Remove all points in the point vector that have the same frame_idx
        points.erase(std::remove_if(points.begin(), points.end(),
                                    [idx](const SRadarPoint& point) {
                                        return point.frame_idx == idx;
                                    }),
                     points.end());

        // If the point vector is empty, remove the voxel from the map
        if (points.empty()) {
            it = m_voxel_map.erase(it);
        } else {
            ++it;
        }
    }
}

// Update the covariance of the voxel in the set
void Voxel2DHashMap::UpdateCovWithSet(std::unordered_set<Voxel, VoxelHash> set_voxel_check)
{
    for(const auto& voxel : set_voxel_check){
        auto it = m_voxel_map.find(voxel);  // Search the voxel in the map
        
        if (it != m_voxel_map.end()) {
            it->second.CalCov(m_config.use_tbb, m_config.voxel_map_use_rcs, m_config.voxel_map_use_range_weight);
        }
    }
}

std::vector<Voxel2DHashMap::Voxel> Voxel2DHashMap::GetSearchVoxel(const SRadarPoint &point) const
{
    std::vector<Voxel2DHashMap::Voxel> voxels;

    Voxel2DHashMap::Voxel voxel = PointToVoxel(point.pose.head<2>(), m_config.voxel_size);
    
    // int voxel_neighbor = (m_config.voxel_search_level == 2) ? 1 : 0;
    int voxel_neighbor = m_config.voxel_search_level;
    int voxel_search_num = (voxel_neighbor == 0) ? 1 : 9; // 3x3 = 9

    voxels.reserve(voxel_search_num);
    for (int i = voxel.x() - voxel_neighbor; i < voxel.x() + voxel_neighbor + 1; ++i) {
        for (int j = voxel.y() - voxel_neighbor; j < voxel.y() + voxel_neighbor + 1; ++j) {
            voxels.emplace_back(i, j);
        }
    }
    return voxels;
}