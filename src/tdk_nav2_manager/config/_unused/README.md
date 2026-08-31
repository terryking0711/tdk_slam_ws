# 未使用的設定檔

這裡的檔案是 `experiment.md` 提到的 planner/controller 比較實驗（Smac2D+DWB、
SmacLattice+MPPI、ThetaStar+MPPI 等）留下的備選設定，**目前沒有任何 launch
檔載入它們**。`nav_launch.py` 只載入 `../tdk_nav2_params.yaml`。

數值可能已過時，且部分與 `tdk_nav2_params.yaml` 目前生效值互相矛盾（例如
`global_costmap.yaml` 的 `inflation_radius: 0.8` vs 生效值 0.30，
`mppi_params.yaml` 的 `vx_max: 0.5` vs 生效值 0.8），不要直接拿來對照生效設定。

`dwb_params.yaml`、`smac_lattice_params.yaml`、`theta_star_params.yaml` 是
空檔案（實驗留下的佔位，從未填入內容）。

要重新啟用其中任何一組設定，必須修改 `nav_launch.py` 讓它載入對應的檔案
（目前是寫死載入 `tdk_nav2_params.yaml` 單一檔案）。
