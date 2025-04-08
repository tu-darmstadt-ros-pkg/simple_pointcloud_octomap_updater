# SimplePointCloudOctomapUpdater

`SimplePointCloudOctomapUpdater` is a more lightweight alternative to MoveIt’s `PointCloudOctomapUpdater` from
`moveit_ros/perception`. It removes the internal self-filtering pipeline, allowing integration with externally
self-filtered point clouds.

---

## 🚀 Motivation

The default `PointCloudOctomapUpdater` performs CPU-based self-filtering, which can become a bottleneck for high-rate
sensors or complex robots. This package allows you to offload filtering to a separate node, potentially using GPU
acceleration .

---

## 🛠️ How It Works

- Subscribes directly to a **pre-filtered point cloud** topic.
- Constructs and updates the octomap using the filtered data.
- Fully compatible with MoveIt’s `occupancy_map_monitor`.


---

## 🧪 Example YAML Config

Use this plugin in your robot's `sensors_3d.yaml` file (e.g., `my_robot_moveit_config/config/sensors_3d.yaml`):

```yaml
# TODO: Switch to this as soon as Mavo's self-filter is fixed
octomap_resolution: 0.05
octomap_frame: world
sensors:
  - front_lidar_pointcloud
  - back_lidar_pointcloud

front_lidar_pointcloud:
    sensor_plugin: occupancy_map_monitor/SimplePointCloudOctomapUpdater
    point_cloud_topic: front_lidar_sensor/self_filter/filtered
    min_range: 0.0
    max_range: 5.0
    point_subsample: 1
    max_update_rate: 50.0

back_lidar_pointcloud:
    sensor_plugin: occupancy_map_monitor/SimplePointCloudOctomapUpdater
    point_cloud_topic: back_lidar_sensor/self_filter/filtered
    min_range: 0.0
    max_range: 5.0
    point_subsample: 1
    max_update_rate: 50.0
