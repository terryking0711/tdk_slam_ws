# TDK 30 Localization & Navigation

## Environment
### pre-installed pkgs and tools
- LiDAR plugin in Gazebo [source](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- ira_laser_tools: laserscan_multi_merger [source](https://github.com/nakai-omer/ira_laser_tools/tree/humble)
- rplidar_ros: S3 LiDAR firmware driver [source](https://github.com/Slamtec/rplidar_ros)
- phidgets_drivers: IMU firmware driver [source](https://github.com/ros-drivers/phidgets_drivers)

### How to setup
1. In local PC:
    ```bash
    git clone -b sim https://github.com/wang-hsiu-cheng/tdk_slam_ws.git
    cd docker && chmod +x run_dev.sh && ./run_dev.sh
    docker exec -it tdk_slam bash
    ```
2. In container:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

### How to setup (Windows)
1. 確認 **VcXsrv（XLaunch）** 在工作列執行，且勾選 `Disable access control`
2. 在 **PowerShell** 啟動容器：
    ```powershell
    cd D:\DIT\ros2_projects\tdk_30th\tdk_slam_ws\docker
    $env:DISPLAY = "host.docker.internal:0.0"
    docker compose up -d
    ```
3. 進入容器並 build：
    ```bash
    docker exec -it tdk_slam bash
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## 模擬建圖 SOP (Windows)

> 每個 Terminal 都需要先執行 `docker exec -it tdk_slam bash` 進入容器

### Terminal 1 — Gazebo 世界
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdk_slam_manager maze_world_launch.py
```

### Terminal 2 — 機器人 + SLAM
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdk_slam_manager sim_spawn_launch.py
```

### Terminal 3 — 鍵盤開車
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Terminal 4 — RViz 視覺化
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```
開啟後手動 Add：
- `Add → By topic → /map → Map → OK`
- `Add → By topic → /scan → LaserScan → OK`
- Fixed Frame 設為 `map`

### 開車按鍵速查

| 按鍵 | 動作 |
|------|------|
| `i` | 直走 |
| `,` | 倒退 |
| `j` | 左轉 |
| `l` | 右轉 |
| `u` | 左前 |
| `o` | 右前 |
| `k` | 停止 |
| `q` / `z` | 加速 / 減速 |
| `w` / `x` | 增加 / 減少線速度 |
| `e` / `c` | 增加 / 減少角速度 |

---

## Localization
### pre-mapping mode
1. modify localization_mode in `sim_spawn_launch.py` (or `spawn_launch.py`)
    - slam toolbox, amcl: `mapping`
    - cartographer: `carto_mapping`
    ```py
    DeclareLaunchArgument('localization_mode', default_value='mapping')
    ```
2. Launch all the programs for mapping (simulation)
    - open simulation world: `ros2 launch tdk_slam_manager maze_world_launch.py`
    - open lidar filter, merger and slam pkgs: `ros2 launch tdk_slam_manager sim_spawn_launch.py`
    - remember to delete robot brfore reopen `sim_spawn_launch.py`: `ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'tdk_robot'}"`
3. Launch all the programs for mapping (real world):
    - open lidar driver, filter, merger and slam pkgs: `ros2 launch tdk_slam_manager spawn_launch.py`
4. Stop mapping and save map
    - save map for slam_toolbox: `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'slam_map_0'}"`
    - save map for nav_amcl: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'amcl_map_0'}}"`
    - save map for catographer:
        1. 停止軌跡: `ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"`
        2. 儲存地圖: `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/tdk/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0.pbstream', include_unfinished_submaps: true}"`
        3. 序列化地圖 (類似 slam_toolbox 的 serialize): `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/tdk/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0.pbstream'}"`
        4. 將地圖轉換成 nav2 可以使用的圖片:
            ```
            ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
                -pbstream_filename /home/tdk/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0.pbstream \
                -map_filestem /home/tdk/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0 \
                -resolution 0.05
            ```

### localization mode
1. modify localization_mode in `sim_spawn_launch.py` (or `spawn_launch.py`)
    - slam toolbox: `slam_toolbox`
    - cartographer: `cartographer`
    - amcl: NOT AVAILABLE
    ```py
    DeclareLaunchArgument('localization_mode', default_value='slam_toolbox')
    ```
2. Launch all the programs for mapping: SAME AS **pre-mapping mode**

### Interact & Visulization
   - `rviz2`: can visualize every ros info (robot pose, tf, lidar scan)
   - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`: move robot around. Can use to create map or test the localization quality

#### notice
- 用teleop玩車的時候小心翻車。

---

## Navigation


---

## File Structure
```bash
├── sensor_dep
│   ├── ira_laser_tools                     # laserscan_multi_merger
│   ├── phidgets_drivers                    # IMU firmware driver
│   └── rplidar_ros                         # S3 LiDAR firmware driver
├── tdk_nav2_manager
└── tdk_slam_manager
    ├── CMakeLists.txt
    ├── package.xml
    ├── cartographer_config
    │   ├── cartographer_2d.lua             # cartographer 參數 (大部分設定都在這)
    │   └── localization.lua                # cartographer 參數 (上層)
    ├── config
    │   ├── amcl_params.yaml                # AMCL 參數
    │   ├── laser_merger_params.yaml        # laser merger 參數
    │   ├── mapper_params_online_async.yaml # slam_toolbox 建圖參數
    │   ├── robot_params.yaml               # 機器人基本規格設定
    │   └── slam_toolbox_params.yaml        # slam_toolbox 參數
    ├── launch
    │   ├── maze_world_launch.py            # 開啟模擬環境
    │   ├── sim_spawn_launch.py             # for 模擬的 launch
    │   └── spawn_launch.py                 # for 真實機器的 launch
    ├── src
    │   └── laser_angle_filter.cpp          # 簡易版的 LiDAR 資訊過濾器
    ├── maps
    │   ├── carto_map_0.pbstream            # cartographer 建出來的 mapping 資訊
    │   ├── carto_map_0.pgm                 # 轉換成 nav2 可以用的格式
    │   ├── carto_map_0.yaml                # 轉換成 nav2 可以用的格式
    │   ├── slam_map_0.data                 # slam_toolbox 建的圖
    │   └── slam_map_0.posegraph            # slam_toolbox 建的圖
    ├── sim
    │   ├── models                          # 物件CAD檔 & 相關轉檔設定 (想知道可以問 yvonne)
    │   └── worlds                          # 把物件匯入 gazebo
    └── urdf
        └── sensors.urdf.xacro              # 機器人關節設定 (想知道可以問 yvonne)
```