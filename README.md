# Radar4VoxMap: Accurate Odometry from Blurred Radar Observations

**Paper:** [Radar4VoxMap: Accurate Odometry from Blurred Radar Observations](https://ieeexplore.ieee.org/document/11128118)

**Authors:** Jiwon Seok, Soyeong Kim, Jaeyoung Jo, Jaehwan Lee, Minseo Jung, and Kichun Jo

**Affiliation:** Hanyang University, Konkuk University

**Code:** [https://github.com/ailab-hanyang/Radar4VoxMap](https://github.com/ailab-hanyang/Radar4VoxMap)

**YouTube:** [Radar4VoxMap demonstration videos](https://www.youtube.com/playlist?list=PL2syYXpEAAZOFc_WLnlx3k0WYlaoEDHwt)

## Abstract

Compared to conventional 3D radar, 4D imaging radar provides additional height data and finer resolution measurements. Moreover, compared to LiDAR sensors, 4D imaging radar is more cost-effective and offers enhanced durability against challenging weather conditions. Despite these advantages, radar-based localization systems face several challenges, including limited resolution, leading to scattered object recognition and less precise localization. Additionally, existing methods that form submaps from filtered results can accumulate errors, leading to blurred submaps and reducing the accuracy of SLAM and odometry. To address these challenges, this paper introduces *Radar4VoxMap*, a novel approach designed to enhance radar-only odometry. The method includes an RCS-weighted voxel distribution map that improves registration accuracy. Furthermore, fixed-lag optimization with the graph is used to optimize both the submap and pose, effectively reducing cumulative errors.

## Key Features

*   **RCS-Weighted Voxel Distribution Map:** Enhances registration accuracy by utilizing Radar Cross Section (RCS) values to weight radar points within voxels.
*   **Graph-based Fixed-Lag Smoothing Optimizer:** Employs a fixed-lag smoothing approach using g2o to optimize a window of recent poses and the associated submap.
*   **Doppler-GICP (D-GICP) for Robust Registration:** Incorporates Doppler velocity measurements alongside geometric information and point/voxel covariances for more robust point cloud registration.
*   **Constant Velocity (CV) Motion Model:** Utilizes a CV model for pose prediction, suitable for typical vehicle motion patterns.

## Dataset: View-of-Delft (VoD)

Radar4VoxMap was primarily evaluated on the **View-of-Delft (VoD) dataset** ([Official Website](https://intelligent-vehicles.org/datasets/view-of-delft/)). This is a publicly available automotive dataset recorded in Delft, the Netherlands, specifically designed for research in autonomous driving and multi-sensor fusion.

*   **Sensor Configuration:** The VoD dataset was collected using a vehicle equipped with a rich sensor suite, including:
    *   **4D Imaging Radars:** Multiple Continental ARS548 sensors, providing dense point clouds with range, azimuth, elevation, Doppler velocity, and Radar Cross Section (RCS) for each point. This is the primary input for Radar4VoxMap.
    *   **LiDAR:** A 64-layer LiDAR sensor for detailed 3D environmental perception and ground truth generation.
    *   **Cameras:** Stereo camera setups for visual data.
*   **Data Characteristics:**
    *   The dataset comprises over 8,600 synchronized frames from all sensors.
    *   It includes extensive 3D bounding box annotations for various road users (over 123,100 total, including pedestrians, cyclists, and cars).
    *   Semantic map information (lanes, sidewalks, etc.) is also provided.
*   **Scenarios:** Data was collected in complex urban traffic environments in Delft, featuring a high density of Vulnerable Road Users (VRUs), which presents significant challenges for perception and localization algorithms.
*   **Access:** The VoD dataset is available for non-commercial research purposes. Access can be requested through a form on the [VoD dataset website](https://intelligent-vehicles.org/datasets/view-of-delft/).
*   **ROS Bag for Testing:**
    To facilitate testing and reproduction of our results, we provide ROS bag files for selected sequences from the VoD dataset. These bags contain the necessary radar topics and ground truth information.
    *   **Download Link for VoD ROS Bags:** `Waiting for permission from the original author.`

## Custom Dataset: bitsensing AFI910 (Test Data)

In addition to the VoD dataset, we provide a sample custom dataset for testing and integration purposes. This dataset was collected using a different sensor setup:

*   **Radar:** bitsensing AFI910 4D Imaging Radar
*   **LiDAR:** Velodyne VLP-32C
*   **GNSS/IMU:** Novatel PwrPak7
*   **Camera:** CANLab Camera

**Important Note:** This custom dataset is provided **solely for testing the integration and functionality** of Radar4VoxMap with different sensor types. The sensor calibration and ground truth localization data for this dataset are **not guaranteed to be accurate**. Use it primarily to verify data flow and compatibility, not for rigorous performance evaluation.

*   **Download Link for Custom Dataset ROS Bags:** [OneDrive](https://1drv.ms/f/s!Ai0drVzacIixsIVHvF_UflIhf1CdhA?e=9PaVcz)
*   **Usage:** To use this dataset:
    To use the bitsensing AFI910 dataset, run the following command:
    ```bash
    roslaunch radar_4_vox_map radar_4_vox_map_afi910.launch
    ```
    *   Refer to `ros/radar_point_converters.hpp` to see how the `afi910` type handles point field mapping.

The comprehensive nature of the VoD dataset, particularly its high-quality 4D radar data and accurate ground truth, makes it an ideal platform for benchmarking radar-based odometry and SLAM systems.

## Dependencies

*   C++17 (or as required)
*   Eigen3
*   PCL (Point Cloud Library)
*   g2o (Graph Optimization Framework)
*   ROS (Robot Operating System) - for using the provided ROS bags and visualization

## Build Instructions

```bash
# Move your catkin workspace
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/ailab-hanyang/Radar4VoxMap.git

cd ..

# Configure and build
catkin_make
source devel/setup.bash
```

## Running

### With View-of-Delft (VoD) Dataset ROS Bags

1.  Download the VoD ROS bag from the link provided above.
2.  Ensure your ROS environment is sourced: `source devel/setup.bash` (if using a ROS workspace).
3.  Launch the odometry node:
    ```bash
    roslaunch radar_4_vox_map radar_4_vox_map.launch
    ```

### Using Custom Radar Datasets

To evaluate Radar4VoxMap with your own custom radar dataset, you will need to:

1.  **Modify Point Conversion:**
    *   The radar point cloud messages from your custom radar might have a different format or topic name.
    *   You will need to adapt the point conversion logic. This typically involves modifying a custom function within the `ros/radar_point_converters` (or a similarly named directory/package responsible for converting raw radar messages to the `SRadarPoint` structure used by the odometry algorithm).
    *   Ensure that your custom conversion function correctly populates all necessary fields: `pose` (x,y,z), `local` (x,y,z in sensor frame if different), `vel` (Doppler velocity), `rcs`, `range`, `azi_angle`, `ele_angle`, `is_static` (if available), and `timestamp`.

2.  **Configure ROS Launch File:**
    *   In your `roslaunch` file (e.g., a copy of `radar_odometry_custom.launch`), you need to specify that you are using a custom radar type.
    *   Set the `radar_type` (or a similar parameter) to `custom`. For example:
        ```xml
        <launch>
            <!-- Other parameters -->
            <param name="radar_type" type="string" value="custom" />
            <param name="custom_radar_topic" type="string" value="/your/custom_radar_topic" />
            <!-- ... other nodes and parameters ... -->

            <node pkg="radar_4_vox_map" type="radar_odometry_node" name="radar_odometry_node" output="screen">
                <!-- Pass parameters to the node -->
            </node>
        </launch>
        ```
    *   You will also need to remap the input radar topic in the launch file to match your custom radar's topic name.

3.  **Data Playback:**
    *   Play your custom radar data, typically from a ROS bag file:
        ```bash
        rosbag play your_custom_radar_data.bag
        ```

By following these steps, you can adapt Radar4VoxMap to process and evaluate data from different radar sensors.

## Citation

If you use Radar4VoxMap in your research, please cite our paper:

**For Radar4VoxMap:**
*(Please update the BibTeX entry with the correct publication details once available.)*

## License

This project is licensed under the Apache 2.0 License - see the `LICENSE` file for details. The View-of-Delft dataset has its own licensing terms, which must be adhered to.

## Acknowledgements

This work was supported in part by a grant from the National Research Foundation of Korea (NRF) grant funded by the Korea government (MSIT). (No.RS-2023-00209252) and the National Research Foundation of Korea (NRF) funded by the Korean government Korean Government Ministry of Science and ICT (MSIT) under grant No. RS-2024-00421129.

We would also like to express our gratitude to the developers and maintainers of the following open-source projects, which provided valuable insights and inspiration for Radar4VoxMap:
*   **Doppler-ICP:** [aevainc/Doppler-ICP](https://github.com/aevainc/Doppler-ICP) - For their pioneering work on incorporating Doppler information into ICP.
*   **KISS-ICP:** [PRBonn/kiss-icp](https://github.com/PRBonn/kiss-icp) - For their robust and efficient LiDAR odometry pipeline.
*   **g2o (General Graph Optimization):** [RainerKuemmerle/g2o](https://github.com/RainerKuemmerle/g2o) - For the versatile graph optimization framework.
*   **VoxelMap:** [hku-mars/VoxelMap](https://github.com/hku-mars/VoxelMap) - For their efficient adaptive voxel mapping method.
*   **glim:** [koide3/glim](https://github.com/koide3/glim) - For their versatile point cloud-based 3D localization and mapping framework.

### Running the 2-D Variant

Radar4VoxMap also provides a lighter **2-D version** (planar assumption) that is useful for applications where roll/pitch variations are negligible.  You can switch between 3-D (default) and 2-D simply by changing the `radar_4_vox_map_type` launch argument.

```bash
# Run the 2-D variant
roslaunch radar_4_vox_map radar_4_vox_map.launch radar_4_vox_map_type:=2d
```

Or, change to `2d` in `ros/launch/radar_4_vox_map_ros.launch`.

```xml
<arg name="radar_4_vox_map_type" default="2d" />
```
