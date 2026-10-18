# TDK 30th — tdk_slam_ws（定位與導航）

TDK 第 30 屆機器人競賽的**定位 + 導航** workspace。負責雙 LiDAR 前處理、Cartographer 定位、TF 樹與 Nav2 導航，並提供「場地座標（world frame）」的機器人位置給主程式使用。

本 ws 是整台機器人軟體的其中一塊，搭配以下 workspace 一起運作：

| Workspace | 負責 |
|-----------|------|
| **tdk_slam_ws**（本 repo） | 感測器前處理、定位、TF、Nav2 |
| [`robot_fsm_v2_ws`](https://github.com/terryking0711/robot_fsm_v2_ws) | 比賽任務 FSM、named pose 導航、DirectTransition 控速、robot_panel 後端 |
| `microros_ws` | STM32 micro-ROS agent（底盤 / 機構），以及 rosbridge、foxglove_bridge 等常駐 bridge |
| `robot_panel_ui` | 瀏覽器控制面板（狀態監控、一鍵啟動比賽） |

- 分支：`tmep_ver`
- 環境：Ubuntu 22.04 / ROS 2 Humble / Docker，CycloneDDS，`ROS_DOMAIN_ID=30`
- 底盤：麥克納姆輪（全向），RPLiDAR S3 ×2（前 / 後）

---

## 目錄

- [系統架構](#系統架構)
- [Docker 快速開始](#docker-快速開始)
- [啟動方式](#啟動方式)
- [場地與地圖設定（field_side）](#場地與地圖設定field_side)
- [TF Tree](#tf-tree)
- [初始定位流程](#初始定位流程)
- [套件與節點](#套件與節點)
- [Nav2 設定](#nav2-設定)
- [進度與已知問題](#進度與已知問題)

---

## 系統架構

### 整體資料流

```mermaid
flowchart LR
    %% ========== 操作介面 ==========
    subgraph UI ["🖥️ 操作介面（瀏覽器）"]
        direction TB
        Panel["<b>robot_panel_ui</b><br>START NORMAL / MIRROR / DIT<br>STOP · RESET STAGE 2/3"]
        Fox["<b>Foxglove / Lichtblick</b><br>地圖 · TF · named_poses"]
    end

    %% ========== 常駐 bridge container ==========
    subgraph BR ["🔌 常駐 container（microros_ws/docker）"]
        direction TB
        RB["rosbridge_websocket<br>:9090"]
        FB["foxglove_bridge<br>:8767"]
        AgF["micro-ROS agent fast<br>/dev/stm32_fast · 1M baud"]
        AgS["micro-ROS agent slow<br>/dev/stm32_slow · 115200"]
    end

    %% ========== 硬體 ==========
    subgraph HW ["🔧 硬體"]
        direction TB
        STMF["STM32 底盤<br>mecanum 控制 + 里程計"]
        STMS["STM32 機構"]
        LF["RPLiDAR S3<br>front"]
        LR["RPLiDAR S3<br>rear"]
    end

    %% ========== tdk_slam_ws：定位 ==========
    subgraph SLAM ["📍 tdk_slam_ws · tdk_slam_manager（spawn_launch.py）"]
        direction TB
        FF["filter_front"]
        FR["filter_rear"]
        MG["laser_merger<br>融合成 360° /scan"]
        OTF["odom_tf_broadcaster<br>odom → base_footprint"]
        CARTO["cartographer_node<br>pure localization<br>map → odom"]
        RPP["robot_pose_publisher<br>/robot_pose（world）"]
        LM["localization_manager<br>初始定位 / 驗證"]
        WTF["static TF<br>world → map"]
    end

    %% ========== tdk_slam_ws：導航 ==========
    subgraph NAV ["🧭 tdk_slam_ws · tdk_nav2_manager（nav_launch.py）"]
        direction TB
        MS["map_server<br>/map"]
        PL["planner_server<br>Smac2D"]
        CT["controller_server<br>MPPI（Omni）"]
        BH["behavior_server<br>spin / backup"]
        VS["velocity_smoother"]
    end

    %% ========== robot_fsm_v2_ws ==========
    subgraph FSM ["🤖 robot_fsm_v2_ws"]
        direction TB
        FL["fsm_launcher<br>（robot_panel_bridge）"]
        MC["robot_fsm_main<br>MissionController"]
        DT["DirectTransition<br>依 /robot_pose 直接控速"]
        NS["navigation_server<br>named pose → Nav2"]
    end

    CFG[("field_config.yaml<br>normal / mirror / dit")]
    MAPS[("maps/<br>map_normal · map_mirror<br>real_map_0 (DIT)")]

    %% ---- UI ----
    Panel <-->|WebSocket| RB
    Fox <-->|WebSocket| FB
    RB -->|"srv: start / stop / reset"| FL
    FL -->|"ros2 launch robot_bringup<br>field_side:=…"| MC

    %% ---- 感測器 ----
    STMF <--> AgF
    STMS <--> AgS
    LF -->|/front/scan| FF --> MG
    LR -->|/rear/scan| FR --> MG
    AgF -->|/odom| OTF
    AgF -->|/odom| CARTO
    MG -->|/scan| CARTO

    %% ---- 地圖 / 場地 ----
    CFG -.->|選地圖| MAPS
    CFG -.->|world_to_map| WTF
    MAPS -->|.pbstream| CARTO
    MAPS -->|.yaml + .pgm| MS

    %% ---- 定位 ----
    CARTO -.->|TF| RPP
    WTF -.->|TF| RPP
    RPP -->|/robot_pose| LM
    RPP -->|/robot_pose| DT
    MC -->|/init_pose_cmd| LM
    LM -->|/init_pose_status| MC
    LM -->|"srv: finish / start_trajectory"| CARTO
    LM -->|"srv: clear costmap"| CT

    %% ---- 導航 ----
    MS -->|/map| PL
    MC --> DT
    MC -->|"action: navigate_to_named_pose"| NS
    NS -->|"action: navigate_to_pose"| PL
    PL -->|path| CT
    CT -->|/cmd_vel_nav| VS
    BH -->|/cmd_vel_nav| VS
    NS -->|"/cmd_vel_nav（終點微調）"| VS

    %% ---- 速度輸出 ----
    VS -->|/cmd_vel| AgF
    DT -->|/cmd_vel| AgF
    MC <-->|"機構指令 / 回授"| AgS

    classDef ui fill:#e3f2fd,stroke:#1e88e5,color:#0d47a1
    classDef hw fill:#fff3e0,stroke:#fb8c00,color:#e65100
    classDef slam fill:#e8f5e9,stroke:#43a047,color:#1b5e20
    classDef nav fill:#f3e5f5,stroke:#8e24aa,color:#4a148c
    classDef fsm fill:#fce4ec,stroke:#d81b60,color:#880e4f
    classDef data fill:#fffde7,stroke:#fbc02d,color:#5d4037
    class Panel,Fox,RB,FB ui
    class STMF,STMS,LF,LR,AgF,AgS hw
    class FF,FR,MG,OTF,CARTO,RPP,LM,WTF slam
    class MS,PL,CT,BH,VS nav
    class FL,MC,DT,NS fsm
    class CFG,MAPS data
```

### 速度命令路徑

底盤只吃一個 `/cmd_vel`，但有兩種來源，由 FSM 依任務階段決定由誰控制：

```mermaid
flowchart LR
    subgraph Nav2Path ["Nav2 導航（長距離、需避障）"]
        CT["MPPI controller"] -->|/cmd_vel_nav| VS
        BH["behavior_server<br>spin / backup"] -->|/cmd_vel_nav| VS
        NS["navigation_server<br>終點微調"] -->|/cmd_vel_nav| VS
        VS["velocity_smoother"]
    end

    subgraph DirectPath ["DirectTransition 控速（短距離、高精度）"]
        RP[/"/robot_pose<br>world frame"/] --> DT["DirectTransition<br>加減速 ramp + 容差判定"]
    end

    VS -->|/cmd_vel| STM["STM32 底盤<br>（micro-ROS fast）"]
    DT -->|/cmd_vel| STM
```

- **Nav2**：所有 Nav2 內部速度（controller、behavior 的 spin/backup）都先 remap 到 `/cmd_vel_nav`，再經 `velocity_smoother` 輸出 `/cmd_vel`。這樣 `/cmd_vel` 只會有一個 Nav2 的 publisher，不會被 recovery 行為繞過 smoother。
- **DirectTransition**（在 `robot_fsm_v2_ws`）：入口對位、轉角、放置點這類「路線固定、要求 cm 級精度」的段落，不走 Nav2，直接依 `/robot_pose` 計算速度。每個目標點的 `max_speed`、`max_accel` / `max_decel`、`xy_tolerance`、`yaw_tolerance_deg`、`settle_cycles` 都寫在 `fsm_side_<side>.yaml`，改完重啟 FSM 就生效。

---

## Docker 快速開始

每個 workspace 都有自己的 `docker/`，可以單獨 build 與執行。本 ws 的 image 已裝好所有相依套件。`src/` 內的第三方套件（`ira_laser_tools`、`rplidar_ros`、`nav2_smac_planner`）都直接放在 repo 中，不需要 submodule。

```bash
# 1. clone（實機使用 tmep_ver 分支）
git clone -b tmep_ver https://github.com/terryking0711/tdk_slam_ws.git
cd tdk_slam_ws/docker

# 2. build image + 啟動 container（第一次約 10~20 分鐘）
./docker_run.sh             # 會自動帶入主機 UID/GID 並開放 X11

# 3. 進入 container
docker exec -it tdk_slam bash

# 4. 在 container 內 build（工作目錄就是 ~/tdk_slam_ws）
colcon build --symlink-install
source install/setup.bash
```

| 項目 | 說明 |
|------|------|
| 快捷指令 | `cb` = build + source，`sw` = source `install/setup.bash` |
| 自動 source | 新開 `docker exec -it tdk_slam bash` 會自動 source `install/setup.bash` |
| GUI（rviz2） | 主機需 `xhost +local:docker`（`docker_run.sh` 已包含） |
| UID 不是 1000 | 請用 `./docker_run.sh`，否則 `build/ install/ log/` 的擁有者會錯 |
| 環境變數 | `ROS_DOMAIN_ID`（預設 30）、`RMW_IMPLEMENTATION`（預設 cyclonedds）可覆蓋 |
| 改 Dockerfile 後 | `docker compose up -d --build`；關閉：`docker compose down` |

> ⚠ `spawn_launch.py` 與 `nav_launch.py` 會讀 `/workspaces/field_config.yaml`（見[場地與地圖設定](#場地與地圖設定field_side)），container 內必須能存取到這個路徑。

### LiDAR 裝置

前後 LiDAR 用 udev symlink 固定裝置名稱，避免插拔後 `/dev/ttyUSB*` 順序對調：

| 裝置 | 路徑 | namespace / frame |
|------|------|-------------------|
| 前 LiDAR | `/dev/rplidar_front` | `/front` · `laser_front` |
| 後 LiDAR | `/dev/rplidar_rear` | `/rear` · `laser_rear` |

---

## 啟動方式

### 前置：micro-ROS 與 bridge

STM32 的 micro-ROS agent（fast / slow 兩條 serial）和 rosbridge、foxglove_bridge 等常駐服務，跑在 `microros_ws/docker` 的獨立 container 裡。它們不必跟著定位 / 導航一起重啟，所以要先啟動。

### 方式一：一鍵啟動（比賽用）

桌面 icon 會在 tmux 裡同時開好定位、導航、`fsm_launcher` 與 named_poses 標記：

| 桌面 icon | 等同指令 | 用途 |
|-----------|----------|------|
| `TDK Loc_Normal` | `start.sh normal` | 比賽 Normal 場地 |
| `TDK Loc_Mirror` | `start.sh mirror` | 比賽 Mirror 場地 |
| `TDK Loc_DIT` | `start.sh dit` | 實驗室測試場地 |
| `TDK Mapping+Nav` | `start_mapping.sh` | Cartographer 建圖 + Nav2 |

前置的 micro-ROS / bridge 對應桌面 icon `TDK MicroROS`（按一次啟動、再按一次關閉）。

接著用 `TDK Robot Panel` 打開 `robot_panel_ui`，確認各項狀態燈正常後，按下與已載入地圖**相同 side** 的 `START`。`fsm_launcher` 會擋下 side 不一致的啟動請求。

再按一次同一個 icon（或換另一個 side）時，會先對所有 pane 送 Ctrl-C 正常關閉，再重新啟動。換場地也是用這個方式。

### 方式二：手動啟動

```bash
# Terminal 1 — LiDAR + 前處理 + TF + 定位
ros2 launch tdk_slam_manager spawn_launch.py localization_mode:=cartographer field_side:=normal

# Terminal 2 — map_server + Nav2
ros2 launch tdk_nav2_manager nav_launch.py field_side:=normal

# Terminal 3 — 主程式（於 robot_fsm_v2_ws）
ros2 launch robot_fsm robot_bringup.launch.py field_side:=normal
```

### Launch 參數

**`spawn_launch.py`**

| 參數 | 預設 | 說明 |
|------|------|------|
| `localization_mode` | `mapping` | 見下表 |
| `field_side` | 空（= `default_side`） | `normal` / `mirror` / `dit`，決定 pbstream 與 world→map |
| `use_sim_time` | `false` | |

| `localization_mode` | 用途 | 狀態 |
|---------------------|------|------|
| `cartographer` | **比賽主定位**：pure localization 讀 pbstream | ✅ 比賽使用 |
| `carto_mapping` | Cartographer 建圖（`/carto_map` 視覺化） | ✅ 可用 |
| `mapping` | slam_toolbox 建圖 | ✅ 可用 |
| `slam_toolbox` | slam_toolbox 定位 | ⚠ 尚未測試 |
| `amcl` | AMCL 定位（備援） | ⚠ 尚未測試 |

**`nav_launch.py`**

| 參數 | 預設 | 說明 |
|------|------|------|
| `field_side` | 空（= `default_side`） | 決定 map_server 地圖與 spin 速度倍率 |
| `map` | 空 | 指定地圖 yaml 完整路徑；留空則依 `field_side` 自動選 |
| `nav_params_file` | `config/tdk_nav2_params.yaml` | Nav2 參數檔 |
| `use_sim_time` | `false` | |

---

## 場地與地圖設定（field_side）

比賽場地分 **Normal / Mirror** 兩側，另有實驗室的 **DIT** 測試場地。所有跟場地有關的設定都集中在 `/workspaces/field_config.yaml`，定位、導航、FSM 三邊讀的是同一份，所以選 side 時只要指定一次：

```mermaid
flowchart TB
    Pick{{"選擇場地<br>field_side = normal / mirror / dit"}}
    CFG[("field_config.yaml<br>sides.SIDE")]
    Pick --> CFG

    CFG -->|map_name| A["spawn_launch.py<br>cartographer 載入 map_name.pbstream"]
    CFG -->|world_to_map| B["spawn_launch.py<br>world → map 靜態 TF<br>localization_manager 座標轉換"]
    CFG -->|map_name| C["nav_launch.py<br>map_server 載入 map_name.yaml"]
    CFG -->|"fsm_side_file<br>mirror_spin_speed_scale"| D["nav_launch.py<br>產生暫存 Nav2 參數<br>放大 spin 角速度 / 加速度"]
    CFG -->|"named_poses_file<br>fsm_side_file"| E["robot_fsm_v2_ws<br>導航點位 · 鏡像指令代號 · 控速參數"]
```

| side | 地圖 | named_poses | FSM 設定 | 備註 |
|------|------|-------------|----------|------|
| `normal` | `map_normal` | `named_poses_normal.yaml` | `fsm_side_normal.yaml` | 比賽 Normal 場地 |
| `mirror` | `map_mirror` | `named_poses_mirror.yaml` | `fsm_side_mirror.yaml` | 比賽 Mirror 場地 |
| `dit` | `real_map_0` | `named_poses_dit.yaml` | `fsm_side_mirror.yaml` | 實驗室場地，控速沿用 mirror |

- `field_config.yaml` 裡的路徑都指向 `src/`，修改後下次 launch 就生效，**不需要重新 colcon build**。
- mirror / dit 會讀 `fsm_side_*.yaml` 的 `mirror_spin_speed_scale`，同時放大 `behavior_server` 與 `velocity_smoother` 的角速度、角加速度限制，平移參數不變。產生的暫存參數檔在 launch 結束時自動刪除。

### 建立新地圖

1. 啟動建圖模式：`start_mapping.sh`，或 `spawn_launch.py localization_mode:=carto_mapping`
2. 遙控機器人跑完整個場地
3. 存檔（`.pbstream` 給 cartographer 定位，`.yaml/.pgm` 給 map_server）：

```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '<ws 絕對路徑>/src/tdk_slam_manager/maps/<name>.pbstream', include_unfinished_submaps: true}"
ros2 run nav2_map_server map_saver_cli -f src/tdk_slam_manager/maps/<name> \
  --ros-args -r map:=/carto_map -p map_subscribe_transient_local:=true
```

4. 在 `field_config.yaml` 把對應 side 的 `map_name` 改成 `<name>`

---

## TF Tree

```mermaid
flowchart TD
    world["<b>world</b><br>場地左下角為原點"]
    map["<b>map</b><br>建圖原點 = 比賽出發點"]
    odom["<b>odom</b><br>里程計座標系"]
    base["<b>base_footprint</b><br>底盤投影中心"]
    lf["laser_front"]
    lr["laser_rear"]

    world -->|"static_transform_publisher<br>field_config.yaml 的 world_to_map"| map
    map -->|"cartographer pure localization<br>低頻全域校正（會跳動）"| odom
    odom -->|"odom_tf_broadcaster（自寫）<br>轉發 /odom pose，使用上位機時間戳<br>※ 原規劃 EKF，暫時停用"| base
    base -->|"robot_state_publisher（URDF）"| lf
    base -->|"robot_state_publisher（URDF）"| lr
```

- **world**：比賽規則、named_poses、FSM 都用場地座標。`/robot_pose` 就是 `world → base_footprint`。
- **map**：Cartographer 的地圖座標。原點在建圖起點，所以每張地圖的 world→map 偏移要在 `field_config.yaml` 裡設好。
- LiDAR 外參寫在 `config/robot_params.yaml`。前後 LiDAR 的 yaw 若有偏差，可在長直走廊靜止時執行 `scripts/calibrate_lidar_yaw.py` 估計修正量。

---

## 初始定位流程

比賽開始前，FSM 要求定位系統把機器人「放」到出發點，並確認定位正確後才開始任務：

```mermaid
sequenceDiagram
    participant FSM as robot_fsm_main
    participant LM as localization_manager
    participant C as cartographer_node
    participant RP as robot_pose_publisher
    participant N as Nav2 costmap

    FSM->>LM: /init_pose_cmd（world frame）
    Note over LM: world → map 座標轉換<br>（減去 world_to_map）
    LM->>C: srv /finish_trajectory（結束舊軌跡）
    LM->>C: srv /start_trajectory（以目標 pose 為起點）
    loop 驗證（timeout 5 s）
        RP-->>LM: /robot_pose
        Note over LM: 誤差 < 0.15 m / 0.15 rad ?
    end
    LM->>N: srv clear global / local costmap
    LM->>FSM: /init_pose_status = true
```

驗證失敗或逾時會回到 IDLE，可重新送 `/init_pose_cmd`。

---

## 套件與節點

| 套件 | 內容 |
|------|------|
| `tdk_slam_manager` | 自寫節點、cartographer / slam_toolbox / AMCL / EKF 設定、URDF、地圖、`spawn_launch.py` |
| `tdk_nav2_manager` | Nav2 參數（多組可切換）、costmap、behavior tree、`nav_launch.py`（含獨立的 map_server） |
| `nav2_smac_planner` | Smac Planner 原始碼（參考 Eurobot 隊伍版本，vendored） |
| `rplidar_ros` | RPLiDAR S3 驅動（vendored） |
| `ira_laser_tools` | 雙 LiDAR 融合（nakai-omer humble fork，vendored） |

### 自寫節點（`tdk_slam_manager`）

| 節點 | 輸入 | 輸出 | 功能 |
|------|------|------|------|
| `laser_angle_filter` | `/front/scan`、`/rear/scan` | `/*/scan_filtered` | 遮掉指定角度區間（車體結構），並濾除 `self_clearance`（0.20 m）以內打到車身的點 |
| `odom_tf_broadcaster` | `/odom` | TF `odom → base_footprint` | 直接轉發 STM32 里程計，改用上位機時間戳，避免與 LiDAR 時間不同步 |
| `robot_pose_publisher` | TF `world → base_footprint` | `/robot_pose` | 輸出場地座標的機器人位置，給 FSM / DirectTransition 使用 |
| `localization_manager` | `/init_pose_cmd`、`/robot_pose` | `/init_pose_status` | 初始定位狀態機（見[初始定位流程](#初始定位流程)） |

### 主要設定檔

| 檔案 | 內容 |
|------|------|
| `config/robot_params.yaml` | 底盤尺寸、LiDAR 外參 |
| `config/laser_merger_params.yaml` | 雙 LiDAR 融合參數 |
| `cartographer_config/cartographer_2d.lua` | Cartographer 建圖 |
| `cartographer_config/localization.lua` | Cartographer pure localization |
| `config/ekf_config.yaml` | EKF（目前停用，保留待修） |

---

## Nav2 設定

目前比賽使用 **Smac2D planner + MPPI controller（Omni motion model）**，適合麥克納姆輪全向移動。主要參數在 `config/tdk_nav2_params.yaml`：

| 項目 | 值 |
|------|----|
| `vx_max` / `vy_max` | 0.8 m/s |
| `wz_max` | 0.6 rad/s |
| `xy_goal_tolerance` | 0.04 m |
| `yaw_goal_tolerance` | 0.03 rad |

`config/` 另外保留了開發期間比較過的組合，可用 `nav_params_file:=` 切換：

| 檔案 | 組合 |
|------|------|
| `tdk_nav2_params.yaml` | **Smac2D + MPPI（比賽使用）** |
| `smac_2d_params.yaml` | Smac2D |
| `smac_lattice_params.yaml` | Smac Lattice |
| `theta_star_params.yaml` | Theta* |
| `mppi_params.yaml` | MPPI |
| `dwb_params.yaml` | DWB |

---

## 進度與已知問題

### ✅ 已完成

- 雙 LiDAR 驅動、角度與車身濾波、360° 融合 pipeline
- STM32 micro-ROS `/odom` → `odom_tf_broadcaster` 發布 `odom → base_footprint`
- Cartographer 建圖與 pure localization，已建立比賽 Normal / Mirror 地圖與 DIT 實驗室地圖
- `field_config.yaml` 統一管理場地：一個 `field_side` 同時決定地圖、world→map、named_poses、FSM 設定
- `localization_manager`：初始定位、驗證、清 costmap、回報 FSM
- `robot_pose_publisher` 輸出場地座標 `/robot_pose`
- Nav2：Smac2D + MPPI（Omni），速度統一經 `velocity_smoother`，mirror / dit 支援 spin 速度倍率
- 與 robot_panel_ui 整合，比賽可一鍵啟動

### ❌ 未完成

- **EKF（robot_localization）停用中**：實機測試時 `/odometry/filtered` 異常，暫以自寫 `odom_tf_broadcaster` 取代；IMU 融合也還沒接上，`ekf_config.yaml` 保留待修
- **slam_toolbox 定位模式未實機測試**：launch 與 `localization_manager` 的 `/initialpose` 介面已寫好
- **AMCL 定位模式未實機測試**
