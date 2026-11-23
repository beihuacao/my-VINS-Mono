# my-VINS-Mono

一个用于学习 VIO 的 ROS 2 增量实现。参考 VINS-Mono 的数据流重新实现核心模块，
但不直接修改原项目源码。

当前环境：Ubuntu 24.04、ROS 2 Jazzy。

## Stage 1：图像与 IMU 时间同步

当前阶段只完成数据入口：

```text
/cam0/image_raw ─┐
                 ├─> 线程安全缓冲区 ─> 按时间戳配对 ─> 同步统计
/imu0 ───────────┘
```

节点不会执行特征跟踪或 VIO 估计。每条日志表示一帧图像已经取得包围其目标时刻
的 IMU 数据。

### 构建

```bash
cd /home/dullfish/projects/VINS-Mono-project/my-VINS-Mono
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动同步节点

终端 1：

```bash
cd /home/dullfish/projects/VINS-Mono-project/my-VINS-Mono
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vio_data_sync euroc_sync.launch.py
```

也可以直接运行并覆盖参数：

```bash
ros2 run vio_data_sync data_sync_node --ros-args \
  -p image_topic:=/cam0/image_raw \
  -p imu_topic:=/imu0 \
  -p time_offset:=0.0
```

成功同步时会输出类似：

```text
sync #12 image=... target=... imus=20 range=[..., ...] bracket=yes dropped=0
```

验收重点：

- `bracket=yes`：IMU 时间范围覆盖图像目标时刻；
- `dropped=0` 或只在启动时丢弃极少量图像；
- 连续播放时没有 `out-of-order` 警告；
- EuRoC 20 Hz 图像、200 Hz IMU 下，每帧通常对应约 10 条新 IMU。

### 数据集格式

现有 `MH_01_easy.bag` 是 ROS 1 bag，ROS 2 Jazzy 的 `ros2 bag play` 不能直接
读取。可以使用 `rosbags` 将需要的两个 topic 转换成 rosbag2 MCAP：

```bash
python3 -m venv /tmp/my_vins_rosbags_venv
/tmp/my_vins_rosbags_venv/bin/pip install rosbags

/tmp/my_vins_rosbags_venv/bin/rosbags-convert \
  --src /home/dullfish/tutorials/pcl_learning_ws/orb_slam3/datasets/MH_01_easy/MH_01_easy.bag \
  --dst /tmp/MH_01_easy_ros2 \
  --dst-storage mcap \
  --dst-typestore ros2_jazzy \
  --include-topic /cam0/image_raw /imu0
```

转换产物约 1.3 GiB，位于 `/tmp`，重启后可能被清理。需要长期保存时，将
`--dst` 改到仓库外的持久目录，不要提交到 Git。

终端 2 播放完整数据：

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/projects/VINS-Mono-project/my-VINS-Mono/MH_01_easy_ros2
```

只播放前5秒进行快速检查：

```bash
ros2 bag play ~/projects/VINS-Mono-project/my-VINS-Mono/MH_01_easy_ros2 \
  --playback-duration 5 \
  --rate 2.0
```

已验证的数据概要：

- 图像：3682条，20 Hz；
- IMU：36820条，200 Hz；
- 前5秒同步100帧，全部 `bracket=yes`、`dropped=0`；
- 稳定阶段每帧包含10条新 IMU，加1条与相邻区间共享的边界 IMU。

## Git 工作流

查看修改：

```bash
git status
git diff
```

每完成一个可验证阶段再提交：

```bash
git add .
git commit -m "feat: add image and IMU synchronization"
git push origin main
```

建议不要提交数据集以及 `build/`、`install/`、`log/` 构建产物。
