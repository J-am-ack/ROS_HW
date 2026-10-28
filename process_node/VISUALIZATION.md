# 录包可视化

运行节点后，再在第二个终端播放录包：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 run process_node process_node
```

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
ros2 bag play /home/ubuntu/ros2_ws/data/rosbag2_2026_10_06-18_13_20/extracted/rosbag2_2026_10_06-18_13_20 --storage sqlite3 --topics /sync_store_node/allsensor /encoder_imu_node/odom
```

播放后出现三个 OpenCV 窗口：

- **Steering Visualization**：录制视频在左，方向盘与状态文字在右。急停时显示原因。
- **Decision Preview**：雷达点和当前命令保持不变时，未来约 4 秒的简化车辆轨迹。
- **Course Map**：已有 Cartographer 地图截图作背景；绿色是录包里程计轨迹，蓝色是把本节点输出命令按简化车辆模型连续积分得到的估算轨迹，紫色是当前命令的短时估算。

录包是开环数据。蓝色与紫色是控制命令的运动估算，不会反过来改变摄像头画面、激光和状态机输入；它们也不代表经过物理仿真验证的车辆轨迹。

当前路线顺序是：第一次右转、第二次右转、第三直行道的左偏和右回 offset、第三次主右转、最后一段直线的路障 S 弯、回到起点附近后的最终右转。最终右转完成后状态进入 `FINISH`。只有第三次主右转完成后才启用路障 S 弯绕行；进入 `AVOID_LEFT` 或 `AVOID_RIGHT` 后依次偏离、通过、反向回正。近距离安全停车在全程有效。

Decision Preview 现在把紫色预测中心线与当前激光回波比较：距离小于 0.52 m 显示 `NEAR`，小于 0.32 m 显示 `COLLISION_RISK`，并圈出最近回波。窗口里的 `events` 是风险片段数；每次新风险或由接近升级为碰撞风险时，节点会输出 `[RISK]` 日志，并默认写入 `/tmp/process_node_risk_events.csv`。可用 `-p risk_log_path:=/指定路径/risks.csv` 修改 CSV 路径。这是基于当前雷达和简化运动模型的**预测风险**，无法证明录包里的车真的撞到了障碍物，也不能仅凭单帧激光点可靠区分墙与独立障碍物；`kind` 取自当前场景分类。

如使用另一张与同一 odom 坐标系对齐的地图，可通过 ROS 参数 `map_image_path` 指定图片，并通过 `map_origin_x_px`、`map_origin_y_px`、`map_pixels_per_meter_x`、`map_pixels_per_meter_y` 调整叠加位置。无图形界面运行时可加 `--ros-args -p visualize:=false`。
