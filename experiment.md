## 實驗流程
1. 雙 LiDAR 掃描預先建圖: slam_toolbox, catographer
    - 項目:
    1. 純 LiDAR 掃描，調整 LiDAR fusion 角度與設定
    2. slam_toolbox: 比較 odometry 回授 & odometry+imu (EKF) 回授
    3. catographer: 比較 imu 回授 & odometry+imu (EKF) 回授
    - 驗證: 地圖形狀的精確度
2. 使用預先建圖結果進行純 LiDAR SLAM 定位: 
    - 比較 amcl, slam_toolbox, catographer 定位效果
    - 驗證:
    1. 靜止時的抖動
    2. 瞬間移動後的回復速度
    3. 遮擋容忍程度
    4. 更新速度與延遲
3. 使用上述較優秀方案進行有回授的 SLAM 定位+導航: 
    - 項目: 
    1. 根據定位方式選擇對應的回授: amcl(odometry), slam_toolbox(odometry), catographer(imu)
    2. 比較 SmacLattice+MPPI, Smac2D+DWB, ThetaStar+MPPI 導航效果
    - 驗證:
    1. 軌跡平滑程度
    2. 機器人是否按照 planner path 走動
    3. 電腦 CPU 使用量
4. 使用雙 LiDAR 進行即時建圖純 LiDAR SLAM 定位: 
    1. 比較 slam_toolbox, catographer 定位效果