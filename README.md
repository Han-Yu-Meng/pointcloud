# PCL 库

依赖 PCL，提供点云采集、预处理以及可视化节点，适合和 FastLIO / ROS 管道自由组合。

## 节点一览

### Source

| 节点 | 说明 | 输出 | Extern |
| --- | --- | --- | --- |
| `cloud_source::random_cloud_source` | 定期生成随机点云，方便调试。 | `pcl::PointCloud<pcl::PointXYZ>::Ptr` | `num_points`、`lower`、`upper`、`interval_ms` |
| `cloud_source::pcd_file_source` | 以固定周期读取 PCD 并发布。 | `pcl::PointCloud<pcl::PointXYZ>::Ptr` | `file_path`、`interval_ms` |

### Process

| 节点 | 功能 | 输入 | 输出 | Extern |
| --- | --- | --- | --- | --- |
| `cloud_process::voxel_filter` | 体素格下采样 | XYZ 点云 | 过滤后点云 | - |
| `cloud_process::sor_filter` | 统计滤波 | XYZ 点云 | 去噪点云 | - |
| `cloud_process::random_filter` | 随机采样固定数量点 | XYZ 点云 | 采样点云 | `num` |
| `cloud_process::pass_through_filter` | 直通滤波 | XYZ 点云 | 裁剪点云 | `field_name`, `min`, `max` |
| `cloud_process::plane_segmentation` | RANSAC 平面提取 | XYZ 点云 | 平面点云 | - |
| `cloud_process::euclidean_cluster_extraction` | 欧式聚类 | XYZ 点云 | 彩色聚类点云 | `tolerance`, `min_size`, `max_size` |
| `cloud_process::pointcloud_accumulator` | 持续累加点云并体素滤波 | XYZ(I) 点云 | 地图/累计点云 | `leaf_size_meters` |

### Visualization

| 节点 | 说明 | 输入 | Extern |
| --- | --- | --- | --- |
| `cloud_visualizer::cloud_visualizer` | 显示 `PointXYZ` 点云 | XYZ 点云 | 窗口标题 |
| `cloud_visualizer::cloud_xyzi_visualizer` | 显示带强度的点云 | XYZI 点云 | 窗口标题 |
| `cloud_visualizer::cloud_rgb_visualizer` | 显示彩色点云 | XYZRGB 点云 | 窗口标题 |
| `cloud_visualizer::mesh_visualizer` | 显示 `pcl::PolygonMesh` | Mesh | 窗口标题 |

以上节点均以 `make_node_list` 暴露，可直接通过 `NewStep(...)` 串接。
