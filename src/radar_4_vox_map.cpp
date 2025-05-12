#include "radar_4_vox_map.hpp"

void Radar4VoxMap::Init(VoxelRcsMapperConfig config)
{
    std::cout<<"[Radar4VoxMap] Init"<<std::endl;

    m_config = config;

    m_graph_optimizer.Init(m_config);

    return;
}

// Receive new points and add them to m_graph_optimizer, then call the functions of m_graph_optimizer
std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> Radar4VoxMap::RunCore(std::vector<RadarPoint> points, const double i_radar_timestamp_sec)
{
    /*
    1. Add the current frame to the node with initial values through CV prediction
        1. Also add EdgeSE3 edges through CV prediction
    2. Perform registration of each node against the voxel map
        1. Update CV prediction edges that changed due to nodes whose positions were modified in the last optimization
        2. Sigma values use the getError function, which is the difference between the graph's initial values and post-graph values from the last optimization
        3. Perform registration
    3. Graph optimization (update node positions)
        1. Fix marginalized nodes to perform fixed lag smoothing
    4. Update the graph based on optimized nodes. Delete nodes that are too far away, starting from the back
    5. Update the voxel map based on optimized nodes
    6. Output the position value of the last node (for visualization and evaluation)
    */
    // std::cout << "[Radar4VoxMap] RunCore" << std::endl;

    auto start_core = std::chrono::steady_clock::now();

    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


    // m_tbb_arena.execute([&] {
    //     m_tbb_group.run([&] {

    // 1. Add the current frame to the node with initial values through CV prediction
    //    1. Also add EdgeSE3 edges through CV prediction
    start = std::chrono::steady_clock::now();
    m_graph_optimizer.AddNewFrame(points, i_radar_timestamp_sec);
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] AddNewFrame executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;

    // 2. Perform registration of each node against the voxel map
    start = std::chrono::steady_clock::now();
    m_graph_optimizer.RunIcpAll();
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] RunIcpAll executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;

    // 3. Graph optimization (update node positions)
    start = std::chrono::steady_clock::now();
    m_graph_optimizer.RunOptimization();
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] RunOptimization executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;

    // 4. Update the graph based on optimized nodes. Delete nodes that are too far away, starting from the back
    start = std::chrono::steady_clock::now();
    m_graph_optimizer.UpdateGraph();
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] UpdateGraph executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;

    // 5. Update the voxel map based on optimized nodes
    start = std::chrono::steady_clock::now();
    m_graph_optimizer.UpdateVoxelMap();
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] UpdateVoxelMap executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;


    // std::cout << "[Radar4VoxMap] Thread Done"<< std::endl;

    // 6. Output the position value of the last node (for visualization and evaluation)
    start = std::chrono::steady_clock::now();
    std::pair<GraphOptimizer::AlgoResultTuple, GraphOptimizer::AlgoResultTuple> pair_latest_oldeset_vertex_info = m_graph_optimizer.GetLastVertexInfo();
    end = std::chrono::steady_clock::now();
    elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    // std::cout << "[Radar4VoxMap] GetLastVertexInfo executed in " << (elapsed_us / 1000.0) << " ms" << std::endl;


    auto end_core = std::chrono::steady_clock::now();
    auto elapsed_core_us = std::chrono::duration_cast<std::chrono::microseconds>(end_core - start_core).count();
    std::cout  << std::fixed << std::setprecision(3) << "[Radar4VoxMap] CORE executed in " << (elapsed_core_us / 1000.0) << " ms" << std::endl;

    // std::cout << "[Radar4VoxMap] RunCore Done" << std::endl;

    return pair_latest_oldeset_vertex_info;
}
