# TDK 30th — tdk_slam_ws（定位與導航）

TDK 30 屆競賽機器人的**定位 + 導航** workspace，與主程式 [`robot_fsm_v2_ws`](https://github.com/terryking0711/robot_fsm_v2_ws) 搭配運作：本 ws 負責感測器前處理、SLAM 定位、TF 樹與 Nav2 導航；FSM ws 負責任務決策並透過 topic / action 呼叫本 ws 的服務。

- 分支：`tmep_ver`（實機用）
- 環境：ROS2 Humble + Docker（`docker/`），CycloneDDS，`ROS_DOMAIN_ID=30`

## Docker 快速開始

所有相依套件都已裝在 image 裡，`src/` 內的套件（含 `ira_laser_tools`、`rplidar_ros`）都直接放在 repo 中，不需要 submodule 或 `rosdep install`。

```bash
# 1. clone + 啟動 container（第一次會 build image，約 10~20 分鐘）
git clone https://github.com/terryking0711/tdk_slam_ws.git
cd tdk_slam_ws/docker
docker compose up -d        # 或 ./docker_run.sh（會自動帶 UID/GID 並開放 X11）

# 2. 進入 container
docker exec -it tdk_slam bash

# 3. 在 container 內 build（工作目錄已是 ~/tdk_slam_ws）
colcon build --symlink-install
source install/setup.bash
```

- 之後開新 terminal `docker exec -it tdk_slam bash` 會自動 source `install/setup.bash`
- 容器內快捷指令：`cb` = build + source，`sw` = source `install/setup.bash`
- GUI（rviz2 / gazebo）：主機先執行 `xhost +local:docker`（`docker_run.sh` 已包含）
- 主機使用者 UID 不是 1000 時請用 `./docker_run.sh`，否則 build 產物的檔案擁有者會不對
- 可用環境變數覆蓋：`ROS_DOMAIN_ID`（預設 30）、`RMW_IMPLEMENTATION`（預設 cyclonedds）
- 修改 Dockerfile 後：`docker compose up -d --build`；關閉：`docker compose down`

---

## 整體系統架構（跨兩個 workspace）

```mermaid
graph TD
    subgraph HW [硬體 / 韌體]
        STM32["STM32 底盤 (micro-ROS)<br>mecanum 控制 + 里程計"]
        LiDARs["RPLiDAR S3 ×2<br>(front / rear)"]
    end

    subgraph Sensor_Data [感測器數據]
        Odom[/ "/odom" /]
        ScanF[/ "/front/scan" /]
        ScanR[/ "/rear/scan" /]
    end

    subgraph Pipeline [前處理 tdk_slam_manager]
        FilterF["filter_front<br>(laser_angle_filter)"]
        FilterR["filter_rear<br>(laser_angle_filter)"]
        Merger["laser_merger<br>(ira_laser_tools，融合為 360°)"]
    end

    subgraph OdomEst [里程計 TF]
        OdomTF["odom_tf_broadcaster（自寫）<br>tf: odom → base_footprint<br>※ EKF 暫時停用（上次測試有問題）"]
    end

    subgraph SLAM_Map [SLAM 建圖模式]
        ST_Map["slam_toolbox mapping<br>(async_slam_toolbox_node)"]
        Carto_Map["cartographer mapping<br>+ occupancy_grid_node → /carto_map<br>（僅視覺化）"]
    end

    subgraph SLAM_Loc [SLAM 定位模式（比賽用）]
        Carto_Loc["cartographer_node<br>pure localization（直接讀 pbstream）<br>tf: map → odom"]
        ST_Loc["slam_toolbox localization<br>⚠ 尚未實機測試"]
        LocMgr["localization_manager<br>(初始化 / 驗證 / 清 costmap)"]
    end

    subgraph Nav [導航 tdk_nav2_manager]
        MapServer["map_server<br>(nav_launch.py) → /map"]
        Nav2["Nav2<br>Smac2D planner + MPPI (Omni)"]
    end

    subgraph Output [全局座標輸出]
        WorldTF["world → map 靜態 TF<br>(0.425, 1.0)"]
        PosePub["robot_pose_publisher<br>→ /robot_pose（world frame）"]
    end

    subgraph FSM_WS [robot_fsm_v2_ws（主程式）]
        Main["robot_fsm main<br>(MissionController)"]
        NavSrv["navigation_server<br>action: /navigate_to_named_pose"]
        StmComm["stm_communication_node<br>/cmd_vel → /mecanum/cmd_vel (20Hz)"]
    end

    Map_file[/ "maps/*.pbstream + *.yaml" /]

    %% 感測器來源
    STM32 -- "micro_ros_agent<br>serial 1M baud" --> Odom
    LiDARs --> ScanF & ScanR

    %% 前處理
    ScanF --> FilterF --> Merger
    ScanR --> FilterR --> Merger

    %% 里程計
    Odom --> OdomTF
    Odom -- "topic 訂閱（remap odom → /odom）" --> Carto_Map & Carto_Loc

    %% SLAM 輸入
    Merger -- "topic: /scan" --> SLAM_Map & SLAM_Loc
    Map_file -- "pbstream" --> Carto_Loc
    Map_file -- "yaml + pgm" --> MapServer

    %% 初始化流程
    Main -- "topic: /init_pose_cmd<br>(world frame)" --> LocMgr
    LocMgr -- "topic: /init_pose_status" --> Main
    LocMgr -- "srv: /finish_trajectory<br>srv: /start_trajectory" --> Carto_Loc
    LocMgr -- "topic: /initialpose" --> ST_Loc
    LocMgr -- "srv: clear costmap" --> Nav2

    %% 定位輸出
    Carto_Loc -- "tf: map → odom" --> PosePub
    WorldTF --> PosePub
    PosePub -- "topic: /robot_pose<br>（初始化驗證用）" --> LocMgr

    %% 導航
    MapServer -- "topic: /map" --> Nav2
    Main -- "action: navigate_to_named_pose" --> NavSrv
    NavSrv -- "action: /navigate_to_pose" --> Nav2
    Nav2 -- "topic: /cmd_vel" --> StmComm
    StmComm -- "topic: /mecanum/cmd_vel" --> STM32
```

---

## TF Tree（實機）

```mermaid
graph TD
    world["world<br>(真實場地左下角為原點)"]
    map["map<br>(建圖起始點為原點 = 比賽出發點)"]
    odom["odom<br>(里程計回授座標系)"]
    base_footprint["base_footprint<br>(底盤投影中心)"]
    laser_front["laser_front"]
    laser_rear["laser_rear"]

    world -- "static_transform_publisher<br>(X: 0.425, Y: 1.0，無旋轉)" --> map
    map -- "cartographer pure localization<br>(較低頻 global 校正，會跳動)" --> odom
    odom -- "odom_tf_broadcaster（自寫）<br>(直接轉發 /odom pose，使用上位機時間戳)<br>※ 原規劃為 ekf_filter_node，暫時停用" --> base_footprint
    base_footprint -- "robot_state_publisher（URDF 靜態）" --> laser_front
    base_footprint -- "robot_state_publisher（URDF 靜態）" --> laser_rear
```

---

## 套件結構

| 套件 | 內容 |
|------|------|
| `tdk_slam_manager` | 自寫節點（`odom_tf_broadcaster`、`laser_angle_filter`、`robot_pose_publisher`、`localization_manager`）、cartographer / slam_toolbox / EKF 設定檔、URDF、地圖、`spawn_launch.py` |
| `tdk_nav2_manager` | Nav2 參數（MPPI / DWB / Smac / ThetaStar 多組可切換）、costmap 設定、behavior tree、`nav_launch.py`（含 map_server） |
| `rplidar_ros` | RPLiDAR S3 驅動（vendored） |
| `ira_laser_tools` | 雙 LiDAR 融合（submodule，nakai-omer humble fork） |

## localization_mode（`spawn_launch.py` 啟動參數）

| mode | 用途 | 狀態 |
|------|------|------|
| `mapping` | slam_toolbox 建圖 | ✅ 可用 |
| `carto_mapping` | cartographer 建圖（`/carto_map` 視覺化） | ✅ 可用 |
| `cartographer` | **比賽主定位**：pure localization 讀 pbstream | ✅ 實機測試中 |
| `slam_toolbox` | slam_toolbox 定位 | ⚠ 尚未測試 |
| `amcl` | AMCL 定位（備援） | ⚠ 尚未測試 |

## 啟動流程（實機）

```bash
# Terminal 1 — 連線到 micro-ROS agent（連 STM32，1M baud）/microros_ws
cd microros_ws/
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6 --baudrate 1000000

# Terminal 2 — scan + TF + 定位
ros2 launch tdk_slam_manager spawn_launch.py localization_mode:=cartographer

# Terminal 3 — Nav2 + map_server
ros2 launch tdk_nav2_manager nav_launch.py

# Terminal 4 — 主程式（於 robot_fsm_v2_ws）
ros2 run robot_fsm robot_fsm_main
```

---

## 目前進度

- ✅ 雙 LiDAR 驅動 + 角度濾波 + 360° 融合 pipeline
- ✅ STM32 micro-ROS `/odom` → `odom_tf_broadcaster` 發布 `odom→base_footprint` TF
- ✅ Cartographer 建圖與 pure localization（pbstream），實機地圖 `real_map_0` 已建立
- ✅ `localization_manager`：接收 FSM `/init_pose_cmd`（world frame），透過 `/finish_trajectory` + `/start_trajectory` 重設 cartographer 軌跡，以 `/robot_pose` 驗證（容差 0.15 m / 0.15 rad），成功後清除 Nav2 costmap 並回報 `/init_pose_status`
- ✅ `world→map` 靜態 TF（0.425, 1.0）+ `robot_pose_publisher` 輸出場地全局座標
- ✅ Nav2：Smac2D + MPPI（Omni，mecanum 適用），map_server 獨立 lifecycle manager

## 未完成 / 已知問題

- ❌ **EKF（robot_localization）停用中**：上次實機測試有問題（`/odometry/filtered` 異常），暫以自寫 `odom_tf_broadcaster` 取代；IMU 融合連帶未接入，`ekf_config.yaml` 保留待修
- ❌ **slam_toolbox 定位模式未測試**（launch 與 `localization_manager` 的 `/initialpose` 介面已寫好）
- ⬜ 比賽場地正式地圖尚未重建（目前為練習場地 `real_map_0`）
