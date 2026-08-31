include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", -- frame where imu locate
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,    -- close when using odometry
  use_odometry = true,           -- open when using odometry
  use_nav_sat = false,
  use_landmarks = false,
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 3e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0


-- Pinpoint odometry 已校正過，精度可信任，權重改回接近官方預設，
-- 平衡 LiDAR 與 Odom 的信任度。過去 occupied_space_weight/translation_weight
-- 遠高於 rotation_weight 的組合，在貼牆的長直牆面會欠約束（aperture problem）：
-- 掃描點沿牆滑動時殘差幾乎不變，優化器會讓 pose 沿牆漂移；rotation_weight 過低
-- 又讓 yaw 抖動被麥輪運動學直接轉成橫向修正指令，兩者合起來就是 cmd_vel 震盪。
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- 回官方預設：0.01 等於允許暴力搜尋（real-time correlative scan matcher）
-- 在整個 search window 內自由亂跑，不受偏離 Odom 的懲罰
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

-- 3. 後端位姿圖優化 (Global SLAM)
-- 提高後端優化對 Odom 約束的權重，讓 Pinpoint 的精度真正被信任
-- (仍維持在官方預設 1e5 以下一個數量級，保留給 LiDAR loop closure 修正空間)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4

-- 官方預設值參考（目前策略以此為基準；occupied_space_weight 與 POSE_GRAPH
-- odometry 權重刻意偏離預設，理由見上方註解）：
-- -- Ceres scan matcher (defaults)
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- -- Real-time correlative scan matcher (defaults)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- -- Pose graph odometry weights (defaults)
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
