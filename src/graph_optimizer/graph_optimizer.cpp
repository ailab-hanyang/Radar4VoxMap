// #include <stdio.h>
#include "graph_optimizer/graph_optimizer.hpp"

GraphOptimizer::GraphOptimizer() {};
GraphOptimizer::~GraphOptimizer() {};

void GraphOptimizer::Init(VoxelRcsMapperConfig config) {
    std::cout << "[GraphOptimizer] Init" << std::endl;
    m_config = config;

    m_voxel_hash_map.Init(m_config);

    /* Initialize g2o */
    //// Generate new g2o
    m_p_g2o_optimizer.reset(new g2o::SparseOptimizer());

    //// Initialize algorithm
    // Initialize Solver
    // CSparse
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>          BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    // typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto                                                              linear_solver = g2o::make_unique<LinearSolverType>();

    std::unique_ptr<g2o::Solver> block_solver;
    block_solver = std::move(g2o::make_unique<BlockSolverType>(std::move(linear_solver)));

    // Initialize algorithm
    g2o::OptimizationAlgorithm* algorithm;

    if ( m_config.optimization_algorithm_type == OptimizationAlgorithmType::GaussNewton ) {
        g2o::OptimizationAlgorithmGaussNewton* algorithm_gauss_newton = new g2o::OptimizationAlgorithmGaussNewton(std::move(block_solver));
        algorithm                                                     = dynamic_cast<g2o::OptimizationAlgorithm*>(algorithm_gauss_newton);
    }
    else {
        g2o::OptimizationAlgorithmLevenberg* algorithm_levenberg = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
        algorithm                                                = dynamic_cast<g2o::OptimizationAlgorithm*>(algorithm_levenberg);
    }

    // Set algorithm
    m_p_g2o_optimizer->setAlgorithm(algorithm);
    m_p_g2o_optimizer->setVerbose(false);

    g2o::ParameterSE3Offset* camera_offset = new g2o::ParameterSE3Offset;
    camera_offset->setId(0);
    m_p_g2o_optimizer->addParameter(camera_offset);

    if ( !m_p_g2o_optimizer->solver() ) {
        std::cerr << "[GraphOptimizer] Error: Optimizer solver is not set properly." << std::endl;
        return;
    }

    return;
}

// add new frame to graph with initial value from CV prediction
void GraphOptimizer::AddNewFrame(const std::vector<RadarPoint>& frame, const double i_radar_timestamp_sec) {
    // std::cout << "[GraphOptimizer] AddNewFrame" << std::endl;
    // 0. 직전 vertex 로드
    Eigen::Matrix4d last_pose = Eigen::Matrix4d::Identity();

    // check if the current frame is the first vertex
    bool        b_is_new_vertex = m_p_g2o_optimizer->vertices().empty();
    VertexType* last_vertex;

    // the current frame is not the first vertex
    if ( b_is_new_vertex == false ) {
        last_vertex = dynamic_cast<VertexType*>(m_p_g2o_optimizer->vertex(GetCurNodeIndex()));

        if ( last_vertex != nullptr ) {
            last_pose = last_vertex->estimateIso3().matrix();
            // std::cout << "[GraphOptimizer] AddNewFrame: Last vertex exist" << std::endl;
        }
    }

    // 2. calculate initial position from CV prediction
    Eigen::Matrix4d cv_prediction = Eigen::Matrix4d::Identity();
    // there must be at least 2 vertices to perform CV prediction
    if ( m_p_g2o_optimizer->vertices().size() >= 2 ) {
        // std::cout<<"[GraphOptimizer] AddNewFrame: Make CV Prediction"<<std::endl;

        VertexType* n_1_vertex = dynamic_cast<VertexType*>(m_p_g2o_optimizer->vertex(GetCurNodeIndex()));
        VertexType* n_2_vertex = dynamic_cast<VertexType*>(m_p_g2o_optimizer->vertex(GetCurNodeIndex() - 1));

        if ( n_1_vertex != nullptr && n_2_vertex != nullptr ) {
            // std::cout<<"[GraphOptimizer] AddNewFrame: Valid last 2 vertex"<<std::endl;
            cv_prediction = GetPredictionModel(n_1_vertex, n_2_vertex, i_radar_timestamp_sec);
        }
        else {
            // std::cout << "[GraphOptimizer] AddNewFrame: Invalid last 2 vertex" << std::endl;
        }
    }
    else{
        // std::cout << "[GraphOptimizer] AddNewFrame: Invalid last 2 vertex" << std::endl;
    }

    Eigen::Matrix4d initial_guess = last_pose * cv_prediction; // CV prediction result in global frame

    // 3. add new vertex with initial position from CV prediction
    VertexType* new_vertex = new VertexType();
    new_vertex->setId(GetNewNodeIndex());

    std::vector<SRadarPoint> converted_frame = ConvertRadarPointsToSRadarPoints(frame, GetCurNodeIndex());
    std::vector<SRadarPoint> voxel_frame     = m_voxel_hash_map.VoxelDownsample(converted_frame, m_config.scan_voxel_size * 1.0);
    std::vector<SRadarPoint> cropped_frame   = CropFrame(voxel_frame, m_config.voxel_map_max_distance, 1.0);

    // calculate point uncertainty
    CalFramePointCov(cropped_frame, m_config.radar_range_variance_m, m_config.radar_azimuth_variance_deg,
                     m_config.radar_elevation_variance_deg);

    new_vertex->timestamp = i_radar_timestamp_sec;
    new_vertex->points    = cropped_frame;
    new_vertex->setEstimate(Eigen::Isometry3d(initial_guess));

    // std::cout<<"[GraphOptimizer] AddNewFrame: Add Vertex of ID: "<< new_vertex->id() <<std::endl;
    m_p_g2o_optimizer->addVertex(new_vertex);

    // add DICP Unary Edge. Measurement is performed in RunIcpAll
    EdgeUnaryICPType* edge_unary_icp = new EdgeUnaryICPType();
    edge_unary_icp->setId(GetNewEdgeIndex()); // set unique ID
    edge_unary_icp->setVertex(0, new_vertex); // connect unary edge to the vertex
    edge_unary_icp->setParameterId(0, 0);

    Eigen::Matrix<double, 6, 6> prior_information = Eigen::Matrix<double, 6, 6>::Identity();
    prior_information(0, 0)                       = 1.0 / (m_config.edge_unary_dicp_std[0] * m_config.edge_unary_dicp_std[0]);
    prior_information(1, 1)                       = 1.0 / (m_config.edge_unary_dicp_std[1] * m_config.edge_unary_dicp_std[1]);
    prior_information(2, 2)                       = 1.0 / (m_config.edge_unary_dicp_std[2] * m_config.edge_unary_dicp_std[2]);
    prior_information(3, 3)                       = 1.0 / (m_config.edge_unary_dicp_std[3] * m_config.edge_unary_dicp_std[3]);
    prior_information(4, 4)                       = 1.0 / (m_config.edge_unary_dicp_std[4] * m_config.edge_unary_dicp_std[4]);
    prior_information(5, 5)                       = 1.0 / (m_config.edge_unary_dicp_std[5] * m_config.edge_unary_dicp_std[5]);
    edge_unary_icp->setInformation(prior_information);
    // Robust Kernel
    edge_unary_icp->setRobustKernel(new g2o::RobustKernelHuber());
    edge_unary_icp->robustKernel()->setDelta(1.0);
    m_p_g2o_optimizer->addEdge(edge_unary_icp);

    // 4. add CV prediction edge between previous vertex and new vertex
    if ( m_p_g2o_optimizer->vertices().size() >= 2 ) {
        // std::cout<<"[GraphOptimizer] AddNewFrame: Add CV Prediction Edge"<<std::endl;

        EdgeBinaryPredictionType* edge_prediction_new = new EdgeBinaryPredictionType();
        edge_prediction_new->setId(GetNewEdgeIndex());
        edge_prediction_new->setVertex(0, last_vertex);
        edge_prediction_new->setVertex(1, new_vertex);
        edge_prediction_new->setParameterId(0, 0);

        edge_prediction_new->setMeasurement(Eigen::Isometry3d(cv_prediction));
        std::vector<double> information(PV_STATE_SIZE);
        information[0]  = 1.0 / (m_config.edge_binary_cv_std[0] * m_config.edge_binary_cv_std[0]);   // x
        information[1]  = 1.0 / (m_config.edge_binary_cv_std[1] * m_config.edge_binary_cv_std[1]);   // y
        information[2]  = 1.0 / (m_config.edge_binary_cv_std[2] * m_config.edge_binary_cv_std[2]);   // z
        information[3]  = 1.0 / (m_config.edge_binary_cv_std[3] * m_config.edge_binary_cv_std[3]);   // vx
        information[4]  = 1.0 / (m_config.edge_binary_cv_std[4] * m_config.edge_binary_cv_std[4]);   // vy
        information[5]  = 1.0 / (m_config.edge_binary_cv_std[5] * m_config.edge_binary_cv_std[5]);   // vz
        information[6]  = 1.0 / (m_config.edge_binary_cv_std[6] * m_config.edge_binary_cv_std[6]);   // qx
        information[7]  = 1.0 / (m_config.edge_binary_cv_std[7] * m_config.edge_binary_cv_std[7]);   // qy
        information[8]  = 1.0 / (m_config.edge_binary_cv_std[8] * m_config.edge_binary_cv_std[8]);   // qz
        information[9]  = 1.0 / (m_config.edge_binary_cv_std[9] * m_config.edge_binary_cv_std[9]);   // wx
        information[10] = 1.0 / (m_config.edge_binary_cv_std[10] * m_config.edge_binary_cv_std[10]); // wy
        information[11] = 1.0 / (m_config.edge_binary_cv_std[11] * m_config.edge_binary_cv_std[11]); // wz
        edge_prediction_new->setInformation(information);

        m_p_g2o_optimizer->addEdge(edge_prediction_new);
    }

    // Estimate velocity from doppler for the first vertex
    if ( m_p_g2o_optimizer->vertices().size() == 1 ) {
        // Estimate velocity from doppler
        Eigen::Vector3d estimated_velocity;
        Eigen::Matrix3d velocity_covariance;
        bool b_success = estimateLocalVelocityFromPoints(new_vertex->points, estimated_velocity, velocity_covariance);
        if ( b_success ) {
            g2o::VectorN<PV_STATE_SIZE> estimated_state = new_vertex->estimateVec();
            estimated_state.block<3, 1>(IDX_VX, 0) = estimated_velocity;
            new_vertex->setEstimate(estimated_state);
        }
        new_vertex->setFixed(true);
    }

    // Add Gravity Align Edge for visualization. so loosly fixed
    if ( m_config.virtual_gravity_align == true ) {
        EdgeUnaryGravityAlignType* edge_gravity_align = new EdgeUnaryGravityAlignType();
        edge_gravity_align->setId(GetNewEdgeIndex());
        edge_gravity_align->setVertex(0, new_vertex);
        edge_gravity_align->setParameterId(0, 0);
        edge_gravity_align->setInformation(m_config.virtual_gravity_align_information);
        m_p_g2o_optimizer->addEdge(edge_gravity_align);
    }

    return;
}

/*
2. Register each node with a prior edge to the voxel map:
    1. Use Sigma values derived from the difference between initial and optimized graph values (getError function)
    2. Perform registration
    3. Add Doppler Unary Edge (initial execution)
*/
void GraphOptimizer::RunIcpAll() {
    // std::cout << "[GraphOptimizer] RunIcpAll for " << m_p_g2o_optimizer->vertices().size() << " vertex" << std::endl;

    bool b_is_new_vertex = m_p_g2o_optimizer->vertices().empty();
    if ( b_is_new_vertex ) {
        // std::cout << "[GraphOptimizer] RunIcpAll: Optimizer is empty, no vertices to process." << std::endl;
        return;
    }

    // Use std::map to sort vertices by ID in ascending order
    auto                                    start = std::chrono::steady_clock::now();
    std::map<int, g2o::HyperGraph::Vertex*> sortedVertices(m_p_g2o_optimizer->vertices().begin(), m_p_g2o_optimizer->vertices().end());
    auto                                    end        = std::chrono::steady_clock::now();
    auto                                    elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[GraphOptimizer] RunIcpAll: Sort vertices executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

    // Iterate through EdgeUnaryICPType edges in the graph and perform ICP
    // Depending on the configuration, ICP may only be performed on the most recent vertex
    auto latest_vertex = sortedVertices.rbegin()->second;
    if ( latest_vertex == nullptr ) {
        std::cout << "[GraphOptimizer] RunIcpAll: CANNOT find latest vertex" << std::endl;
        return; // Prevent errors
    }

    // Search all edges in the graph
    for ( auto iter_edge = m_p_g2o_optimizer->edges().begin(); iter_edge != m_p_g2o_optimizer->edges().end(); ++iter_edge ) {
        auto edge_unary_icp = dynamic_cast<EdgeUnaryICPType*>(*iter_edge);
        // Only process EdgeUnaryICPType edges
        if ( edge_unary_icp != nullptr ) {
            // If icp_all_node option is disabled, only process the latest node
            if ( m_config.icp_all_node == false && edge_unary_icp->vertex(0)->id() != GetCurNodeIndex() ) {
                continue;
            }
            // Find the vertex connected to this unary ICP edge
            int         iter_vertex_id = edge_unary_icp->vertex(0)->id();
            VertexType* iter_vertex    = dynamic_cast<VertexType*>(sortedVertices.find(iter_vertex_id)->second);
            if ( iter_vertex == nullptr ) {
                std::cout << "[GraphOptimizer] RunIcpAll: CANNOT find iter vertex" << std::endl;
                continue; // Prevent errors
            }

            // Skip fixed vertices as they don't need ICP
            if ( iter_vertex->fixed() == true ){
                continue;
            }

            // Extract initial guess from the current vertex's estimated value
            // When optimization is disabled:
            Eigen::Matrix4d initial_guess = iter_vertex->estimateIso3().matrix();

            // Find the previous vertex
            auto prev_vertex_iter = sortedVertices.find(iter_vertex_id - 1);
            // If previous vertex exists, proceed. If not, this is the first node in the graph, no need for ICP
            if ( prev_vertex_iter == sortedVertices.end() ) {
                // No previous vertex, so this must be the first vertex. No need for ICP
                continue; 
            }
            auto prev_vertex = dynamic_cast<VertexType*>(prev_vertex_iter->second);
            if ( prev_vertex == nullptr ) { // If no previous vertex, this is the first vertex in the graph. Skip ICP
                std::cout << "[GraphOptimizer] RunIcpAll: DO NOT RUN FOR first vertex" << std::endl;
                continue;
            }
            Eigen::Matrix4d last_pose = prev_vertex->estimateIso3().matrix();

            double dt          = iter_vertex->timestamp - prev_vertex->timestamp;
            double trans_sigma = m_config.initial_trans_threshold;
            double vel_sigma   = m_config.initial_vel_threshold;

            // TODO: Calculate trans_sigma from the difference between vertex velocity and CV prediction edge

            trans_sigma = std::max(trans_sigma, 1.0);
            vel_sigma   = std::max(vel_sigma, 2.0);

            start = std::chrono::steady_clock::now();
            std::vector<SRadarPoint> frame_processed;
            Eigen::Matrix4d          new_pose = edge_unary_icp->RunRegister(iter_vertex->points, frame_processed, m_voxel_hash_map,
                                                                            initial_guess, last_pose, dt, trans_sigma, vel_sigma, m_config);

            end        = std::chrono::steady_clock::now();
            elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            // std::cout << "[GraphOptimizer] RunIcpAll: RunRegister executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

            // Update the points attribute of the vertex
            iter_vertex->points = frame_processed;

            // Set the measurement for the DICP unary edge
            edge_unary_icp->setMeasurement(Eigen::Isometry3d(new_pose));

            // If optimization is disabled, directly apply registration results to the node
            // TODO: Add direct velocity input to Prediction
            if ( m_config.run_optimization == false ) {
                iter_vertex->setEstimate(Eigen::Isometry3d(new_pose));
            }

            // Create Doppler Unary Edge for the latest vertex on first execution
            if ( iter_vertex->id() == GetCurNodeIndex() && m_config.use_doppler_unary_edge == true ) {
                EdgeUnaryDopplerType* edge_unary_doppler = new EdgeUnaryDopplerType();
                // Calculate measurement and information in one step
                if ( edge_unary_doppler->setPointMeasurementAndInformation(frame_processed) == true ) {
                    edge_unary_doppler->setId(GetNewEdgeIndex());
                    edge_unary_doppler->setVertex(0, iter_vertex);
                    edge_unary_doppler->setParameterId(0, 0);
                    edge_unary_doppler->setInformationScale(m_config.edge_unary_doppler_information_scale);
                    m_p_g2o_optimizer->addEdge(edge_unary_doppler);
                }
            }
        }
    }
}

void GraphOptimizer::RunOptimization() {
    // std::cout << "[GraphOptimizer] RunOptimization" << std::endl;
    if ( m_config.run_optimization == false ) {
        std::cout << "[GraphOptimizer] Option. No Optimization" << std::endl;
        return;
    }

    if ( m_p_g2o_optimizer->vertices().size() <= 1 ) {
        // std::cout << "[GraphOptimizer] RunOptimization: No vetex to optimize." << std::endl;
        return;
    }
    // std::cout << "[GraphOptimizer] RunOptimization: Vertex num: " << m_p_g2o_optimizer->vertices().size() << std::endl;

    // Call optimization if needed
    m_p_g2o_optimizer->initializeOptimization();
    m_p_g2o_optimizer->optimize(m_config.optimization_num);

    return;
}

void GraphOptimizer::UpdateGraph() {
    // std::cout << "[GraphOptimizer] UpdateGraph" << std::endl;

    if ( m_p_g2o_optimizer->vertices().empty() == true ) {
        // std::cout << "[GraphOptimizer] UpdateGraph: No vetex to optimize." << std::endl;
        return;
    }

    // Store vertices sorted by ID in a map
    std::map<int, g2o::HyperGraph::Vertex*> sortedVertices(m_p_g2o_optimizer->vertices().begin(), m_p_g2o_optimizer->vertices().end());

    // Use reverse iterator to search from highest ID
    auto lastest_vertex_iter = sortedVertices.rbegin();
    auto latest_vertex       = dynamic_cast<VertexType*>(lastest_vertex_iter->second);

    if ( latest_vertex == nullptr ) {
        // std::cout << "[GraphOptimizer] UpdateGraph: last vertex invalid!" << std::endl;
        return;
    }

    // Position of the last vertex
    Eigen::Vector3d lateset_position = latest_vertex->estimateIso3().translation();

    // If there is only one vertex, fix it and return (first vertex fixed)
    if ( m_p_g2o_optimizer->vertices().size() == 1 ) {
        return;
    }

    // Variable to store the index of the last deleted vertex
    int last_deleted_index = -1;
    int i_node_count       = 0;
    for ( auto it = sortedVertices.rbegin(); it != sortedVertices.rend(); ++it ) {
        auto iter_vertex = dynamic_cast<VertexType*>(it->second);
        if ( iter_vertex == nullptr ) {
            continue;
        }
        i_node_count++;

        // Position of the current vertex
        Eigen::Vector3d iter_position = iter_vertex->estimateIso3().translation();

        // Calculate the distance between the last vertex and the current vertex
        double distance = (iter_position - lateset_position).norm();

        if ( distance > m_config.node_erase_distance || i_node_count > m_config.vertex_num_max ) {
            // If a vertex that satisfies the condition is found, remove all previous vertices

            for ( auto remove_it = sortedVertices.begin(); remove_it != it.base(); ++remove_it ) {
                // Remove all edges connected to the vertex
                std::vector<g2o::OptimizableGraph::Edge*> edges_to_remove;
                for ( auto edge : remove_it->second->edges() ) {
                    edges_to_remove.push_back(static_cast<g2o::OptimizableGraph::Edge*>(edge));
                }
                for ( auto edge : edges_to_remove ) {
                    m_p_g2o_optimizer->removeEdge(edge);
                }

                m_p_g2o_optimizer->removeVertex(remove_it->second);
                last_deleted_index = remove_it->first; // Store the index of the last deleted vertex
            }
            break;                                     // Exit the loop after deleting the vertex
        }
    }
    // Traverse from the deleted vertex index to the latest (highest ID)
    for ( auto it = sortedVertices.upper_bound(last_deleted_index); it != sortedVertices.end(); ++it ) {
        auto it_vertex = dynamic_cast<VertexType*>(it->second);

        // If the vertex exists
        if ( it_vertex != nullptr ) {
            int it_vertex_id         = it->first;
            // If the vertex is the last one or more than optimization_vertex_num_max vertices away from the latest vertex, fix it
            int last_fixed_vertex_id = GetCurNodeIndex() - m_config.optimization_vertex_num_max;
            if ( it_vertex_id <= last_deleted_index + 2 || it_vertex_id < last_fixed_vertex_id ) {
                it_vertex->setFixed(true);
            }
        }
    }
}

void GraphOptimizer::UpdateVoxelMap() {
    // std::cout << "[GraphOptimizer] UpdateVoxelMap" << std::endl;

    // voxel map 초기화
    auto start = std::chrono::steady_clock::now();
    m_voxel_hash_map.Clear();
    auto end        = std::chrono::steady_clock::now();
    auto elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] UpdateVoxelMap  Clear executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

    // Add points to the voxel map for all vertices
    start                                  = std::chrono::steady_clock::now();
    Eigen::Vector3d last_keyframe_position = Eigen::Vector3d::Zero();
    for ( const auto& v_pair : m_p_g2o_optimizer->vertices() ) {
        auto iter_vertex = dynamic_cast<VertexType*>(v_pair.second);
        // Check if the vertex is valid
        if ( iter_vertex ) {
            auto cur_position = iter_vertex->estimateIso3();
            double distance    = (cur_position.translation() - last_keyframe_position).norm();
            bool   is_keyframe = false;
            if ( iter_vertex->fixed() == false ) {
                is_keyframe = true;
            }
            else if ( distance > m_config.fixed_vertex_to_map_keyframe_distance ) {
                is_keyframe = true;
            }
            if ( is_keyframe == true ) {
                m_voxel_hash_map.Update(iter_vertex->points, cur_position.matrix());
                last_keyframe_position = cur_position.translation();
            }
        }
    }
    end        = std::chrono::steady_clock::now();
    elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] UpdateVoxelMap  Add executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;

    // Calculate Cov for all valid voxels
    start = std::chrono::steady_clock::now();
    for ( auto& pair : m_voxel_hash_map.m_voxel_map ) {
        auto& voxel_block = pair.second;
        if ( voxel_block.is_changed == true ) {
            voxel_block.CalCov(m_config.use_tbb, m_config.voxel_map_use_rcs, m_config.voxel_map_use_range_weight);
            voxel_block.is_changed = false;
        }
    }
    end        = std::chrono::steady_clock::now();
    elapsed_ns = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] UpdateVoxelMap  CalCov executed in " << (elapsed_ns / 1000.0) << " ms" << std::endl;
}

GraphOptimizer::PairAlgoResultTuple GraphOptimizer::GetLastVertexInfo() {
    AlgoResultTuple latest_node = std::make_tuple(std::vector<SRadarPoint>(), Eigen::Matrix4d::Identity(), 0.0);
    AlgoResultTuple oldest_node = std::make_tuple(std::vector<SRadarPoint>(), Eigen::Matrix4d::Identity(), 0.0);

    // std::cout << "[GraphOptimizer] GetLastVertexInfo" << std::endl;
    if ( m_p_g2o_optimizer->vertices().empty() == true ) {
        // std::cout << "[GraphOptimizer] GetLastVertexInfo: Graph empty. Return empty" << std::endl;
        return std::make_pair(latest_node, oldest_node);
    }
    else {
        // std::cout << "[GraphOptimizer] GetLastVertexInfo: Last vertex id: " << GetCurNodeIndex() << std::endl;
    }

    VertexType* lastest_vertex = dynamic_cast<VertexType*>(m_p_g2o_optimizer->vertex(GetCurNodeIndex()));
    if ( lastest_vertex == nullptr ) {
        // std::cout << "[GraphOptimizer] GetLastVertexInfo: Fail to cast VertexType. Return empty" << std::endl;
        return std::make_pair(latest_node, oldest_node);
    }
    latest_node = std::make_tuple(lastest_vertex->points, lastest_vertex->estimateIso3().matrix(), lastest_vertex->timestamp);

    // Search all vertices in the graph to find the smallest ID
    int         min_id        = std::numeric_limits<int>::max();
    VertexType* oldest_vertex = nullptr;

    for ( const auto& v_pair : m_p_g2o_optimizer->vertices() ) {
        auto vertex = dynamic_cast<VertexType*>(v_pair.second);
        if ( vertex != nullptr && v_pair.first < min_id && vertex->fixed() == false ) {
            min_id        = v_pair.first;
            oldest_vertex = vertex;
        }
    }
    if ( oldest_vertex != nullptr ) {
        oldest_node = std::make_tuple(oldest_vertex->points, oldest_vertex->estimateIso3().matrix(), oldest_vertex->timestamp);
    }

    return std::make_pair(latest_node, oldest_node);
}

std::tuple<std::vector<SVisVertex>, std::vector<SVisEdgeBinary>, std::vector<SVisEdgeUnary>> GraphOptimizer::GetAllGraphElements() const {
    std::vector<SVisVertex>     o_vec_vertex;
    std::vector<SVisEdgeBinary> o_vec_binary_edge;
    std::vector<SVisEdgeUnary>  o_vec_unary_edge;

    // Traverse all vertices to convert to SVisVertex
    for ( const auto& v_pair : m_p_g2o_optimizer->vertices() ) {
        auto vertex = dynamic_cast<VertexType*>(v_pair.second);
        if ( vertex ) {
            SVisVertex vis_vertex;
            vis_vertex.pose = vertex->estimateIso3().matrix();
            vis_vertex.covariance = Eigen::Matrix<double, 6, 6>::Identity(); // TODO: Calculate covariance
            vis_vertex.id         = vertex->id();
            o_vec_vertex.push_back(vis_vertex);
        }
    }

    // Traverse all edges to convert to SVisEdgeBinary and SVisEdgeUnary
    for ( const auto& e : m_p_g2o_optimizer->edges() ) {
        if ( auto edge_binary = dynamic_cast<EdgeBinaryPredictionType*>(e) ) {
            SVisEdgeBinary vis_edge_binary;
            auto           v_start = dynamic_cast<VertexType*>(edge_binary->vertex(0));
            auto           v_end   = dynamic_cast<VertexType*>(edge_binary->vertex(1));
            if ( v_start && v_end ) {
                vis_edge_binary.vertex_pose_start = v_start->estimateIso3().matrix();
                vis_edge_binary.vertex_pose_end   = v_end->estimateIso3().matrix();
                vis_edge_binary.measurement = edge_binary->measurement().matrix();
                vis_edge_binary.information = edge_binary->informationM6();
                vis_edge_binary.id       = edge_binary->id();
                vis_edge_binary.id_start = v_start->id();
                vis_edge_binary.id_end   = v_end->id();
                o_vec_binary_edge.push_back(vis_edge_binary);
            }
        }
        else if ( auto edge_unary = dynamic_cast<EdgeUnaryICPType*>(e) ) {
            SVisEdgeUnary vis_edge_unary;
            auto          v_start = dynamic_cast<VertexType*>(edge_unary->vertex(0));
            if ( v_start ) {
                vis_edge_unary.vertex_pose = v_start->estimateIso3().matrix();
                vis_edge_unary.measurement = edge_unary->measurement().matrix();
                vis_edge_unary.information = edge_unary->information();
                vis_edge_unary.id          = edge_unary->id();
                vis_edge_unary.id_start    = v_start->id();
                o_vec_unary_edge.push_back(vis_edge_unary);
            }
        }
    }

    return std::make_tuple(o_vec_vertex, o_vec_binary_edge, o_vec_unary_edge);
}

std::vector<SVisEdgeUnaryDoppler> GraphOptimizer::GetVertexDopplerVel() const {
    std::vector<SVisEdgeUnaryDoppler> o_vec_unary_edge_doppler;
    // Traverse all edges to convert to SVisEdgeUnaryDoppler
    for ( const auto& e : m_p_g2o_optimizer->edges() ) {
        if ( auto edge_unary = dynamic_cast<EdgeUnaryDopplerType*>(e) ) {
            SVisEdgeUnaryDoppler vis_edge_unary;
            auto                 v_start = dynamic_cast<VertexType*>(edge_unary->vertex(0));
            if ( v_start ) {
                vis_edge_unary.vertex_pose = v_start->estimateIso3().matrix();
                vis_edge_unary.velocity    = edge_unary->measurement().matrix();
                vis_edge_unary.information = edge_unary->information();
                vis_edge_unary.id          = edge_unary->id();
                vis_edge_unary.id_start    = v_start->id();
                o_vec_unary_edge_doppler.push_back(vis_edge_unary);
            }
        }
    }
    return o_vec_unary_edge_doppler;
}

Eigen::Matrix4d GraphOptimizer::GetPredictionModel(const VertexType* n_1_vertex, const VertexType* n_2_vertex, double cur_timestamp) const {
    Eigen::Matrix4d pred = Eigen::Matrix4d::Identity();

    if ( !n_1_vertex || !n_2_vertex )
        return pred;

    double last_dt = n_1_vertex->timestamp - n_2_vertex->timestamp;
    double cur_dt  = cur_timestamp - n_1_vertex->timestamp;

    // Calculate the delta pose between T-2 and T-1
    Eigen::Matrix4d delta_pose = n_2_vertex->estimateIso3().matrix().inverse() * n_1_vertex->estimateIso3().matrix();

    // Extract linear and angular components from delta_pose
    Eigen::Vector3d delta_translation = delta_pose.block<3, 1>(0, 3);
    Eigen::Matrix3d delta_rotation    = delta_pose.block<3, 3>(0, 0);

    // Calculate linear and angular velocities
    Eigen::Vector3d   linear_velocity = delta_translation / last_dt;
    Eigen::AngleAxisd delta_angle_axis(delta_rotation);
    Eigen::Vector3d   angular_velocity = delta_angle_axis.angle() * delta_angle_axis.axis() / last_dt;

    // Predict the translation component at the current timestamp
    Eigen::Vector3d predicted_translation = linear_velocity * cur_dt;

    // Predict the rotation component at the current timestamp
    Eigen::AngleAxisd predicted_angle_axis(angular_velocity.norm() * cur_dt, angular_velocity.normalized());
    Eigen::Matrix3d   predicted_rotation = predicted_angle_axis.toRotationMatrix();

    // Form the predicted transform
    pred.block<3, 3>(0, 0) = predicted_rotation;
    pred.block<3, 1>(0, 3) = predicted_translation;

    return pred;
}

Eigen::Isometry3d GraphOptimizer::CalculateEdgeError(VertexType* vertex_cur, VertexType* vertex_prev, EdgeBinaryPredictionType* edge) {
    // Get the measurement of the edge
    Eigen::Isometry3d measurement = edge->measurement();

    // Calculate the actual relative transform between the two vertices
    Eigen::Isometry3d relative_transform = vertex_prev->estimateIso3().inverse() * vertex_cur->estimateIso3();

    // Calculate the difference between the measurement and the actual relative transform
    Eigen::Isometry3d error_transform = measurement.inverse() * relative_transform;

    return error_transform;
}

double GraphOptimizer::GetTransAdaptiveThreshold() {
    if ( !HasMoved() ) {
        return m_config.initial_trans_threshold;
    }
    return m_adaptive_threshold.ComputeTransThreshold();
}

double GraphOptimizer::GetVelAdaptiveThreshold() {
    if ( !HasMoved() ) {
        return m_config.initial_vel_threshold;
    }
    return m_adaptive_threshold.ComputeVelThreshold();
}

bool GraphOptimizer::HasMoved() {
    if ( m_poses.empty() )
        return false;
    const double motion = (m_poses.front().inverse() * m_poses.back()).block<3, 1>(0, 3).norm();
    return motion > 5.0 * m_config.min_motion_threshold;
}