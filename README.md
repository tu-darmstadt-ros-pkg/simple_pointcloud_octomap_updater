# SimplePointCloudOctomapUpdater

`SimplePointCloudOctomapUpdater` is a lightweight alternative to MoveIt’s standard `PointCloudOctomapUpdater` (from
`moveit_ros/perception`).

It is designed for two main purposes:

1. **External Filtering:** It removes the internal CPU-based self-filtering pipeline, allowing you to use externally
   filtered (e.g., GPU-accelerated) point clouds.
2. **Direct Obstacle Querying:** It provides a thread-safe, parallelized service to raycast against the internal Octomap
   for distance measurements.

---

## 🚀 Motivation

The default MoveIt updater performs self-filtering on the CPU, which can become a bottleneck for high-rate sensors. This
plugin offloads filtering to a separate node. Additionally, it exposes the internal Octomap representation via a ROS 2
service, allowing other nodes to query "distance to obstacle" without needing their own copy of the map.

---

## 🛠️ Features

- **Lightweight:** Subscribes directly to a **pre-filtered point cloud** topic.
- **Thread-Safe:** Implements read/write locking on the Octomap to safely handle simultaneous point cloud updates and
  service queries.
- **Parallel Execution:** Service requests are handled in a **Reentrant Callback Group**, allowing multiple distance
  queries to be processed simultaneously without blocking the main update loop.
- **Dynamic Parameters:** Critical parameters (ranges, subsampling, timeouts) can be reconfigured at runtime without
  restarting the node.
- **Local Octomap Visualization:** Publishes a cropped (and optionally coarsened) occupied-only octomap around the
  robot for low-bandwidth operator visualization.

---

## 🧪 Example YAML Config

Add this plugin to your robot's `sensors_3d.yaml` (e.g., `my_robot_moveit_config/config/sensors_3d.yaml`):

```yaml
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
  # Optional: TF lookup timeout for service calls (default: 0.5s)
  tf_timeout: 0.5

back_lidar_pointcloud:
  sensor_plugin: occupancy_map_monitor/SimplePointCloudOctomapUpdater
  point_cloud_topic: back_lidar_sensor/self_filter/filtered
  min_range: 0.0
  max_range: 5.0
  point_subsample: 1
  max_update_rate: 50.0

```

---

## ⚙️ Parameters

The following parameters are dynamically reconfigurable. Ranges are validated at runtime (e.g., you cannot set
`min_range` >= `max_range`).

| Parameter           | Type   | Default    | Description                                              |
|---------------------|--------|------------|----------------------------------------------------------|
| `point_cloud_topic` | string | *required* | Topic name for the input point cloud.                    |
| `max_update_rate`   | double | 0.0        | Max Hz to process clouds (0.0 = as fast as possible).    |
| `point_subsample`   | int    | 1          | Process only every *n*-th point (1 = process all).       |
| `min_range`         | double | 0.0        | Ignore points closer than this distance.                 |
| `max_range`         | double | inf        | Ignore points further than this distance.                |
| `tf_timeout`        | double | 0.5        | Max time to wait for TF transforms during service calls. |
| `publish_frequency` | double | 0.0        | Hz for publishing the full octomap (0.0 = disabled).     |
| `octomap_topic`     | string | `octomap_binary` | Topic for the full octomap publisher.              |

---

## 📡 Local Octomap Visualization

For remote operation over constrained links (e.g., WiFi in the field), publishing the full octomap is wasteful. The
plugin can instead publish a **local, occupied-only** octomap: only occupied voxels inside a box centered on the robot
are extracted, optionally coarsened to a lower resolution, and published as a regular `octomap_msgs/msg/Octomap`
(binary encoding). Typical message sizes are a few KB instead of the full map.

- **Lazy:** Nothing is extracted or published while the topic has no subscribers.
- **Robot-centric:** The crop box follows the robot via TF. If the transform is temporarily unavailable, the cycle is
  skipped (throttled warning).
- **Stock RViz:** View with the `octomap_rviz_plugins/OccupancyGrid` display (package `octomap-rviz-plugins`) on the
  configured topic — no custom operator-side software needed.

| Parameter               | Type   | Default         | Description                                                        |
|-------------------------|--------|-----------------|--------------------------------------------------------------------|
| `local_viz.frequency`   | double | 0.0             | Publish rate in Hz (0.0 = feature disabled).                       |
| `local_viz.topic`       | string | `local_octomap` | Output topic (`octomap_msgs/msg/Octomap`, binary).                 |
| `local_viz.robot_frame` | string | `base_link`     | TF frame whose origin centers the crop box.                        |
| `local_viz.range_xy`    | double | 5.0             | Half-extent of the crop box in x/y (map frame), in meters.         |
| `local_viz.range_z`     | double | 2.0             | Half-extent of the crop box in z (map frame), in meters.           |
| `local_viz.resolution`  | double | 0.0             | Output voxel size in meters (0.0 = native octomap resolution).     |

### Example

```yaml
front_lidar_pointcloud:
  sensor_plugin: occupancy_map_monitor/SimplePointCloudOctomapUpdater
  point_cloud_topic: front_lidar_sensor/self_filter/filtered
  # ... other parameters ...
  local_viz:
    frequency: 2.0       # 2 Hz is plenty for operator situational awareness
    robot_frame: base_link
    range_xy: 5.0
    range_z: 2.0
    resolution: 0.1      # coarsen from e.g. 0.05 native resolution to save bandwidth
```

---

## ⏸️ Enable/Disable Octomap Service

The plugin advertises an `/enable_octomap` service to enable or disable point cloud integration at runtime.
When disabled, the octomap is cleared and no new point clouds are integrated. When re-enabled, integration resumes normally.

* **Service Type:** `std_srvs/srv/SetBool`
* **`data: true`** — Enable integration.
* **`data: false`** — Disable integration and clear the octomap.

### CLI Example

```bash
# Disable octomap (clears and stops integration)
ros2 service call /enable_octomap std_srvs/srv/SetBool "{data: false}"

# Re-enable octomap
ros2 service call /enable_octomap std_srvs/srv/SetBool "{data: true}"
```

---

## 🔍 Get Distance to Obstacle Service

The plugin advertises a service `/get_distance_to_obstacle` that allows you to cast rays in the current Octomap.

* **Service Type:** `hector_worldmodel_msgs/srv/GetDistanceToObstacle`
* **Parallelism:** This service runs in a `Reentrant` callback group. Multiple nodes can call this service
  simultaneously; they will not block each other or the point cloud integration.

### Request

```yaml
point:
  header:
    frame_id: "lidar_frame"  # The frame the ray starts from
    stamp: "..."             # Time of the query (affects TF lookup)
  point:
    x: 1.0  # Direction vector X
    y: 0.0  # Direction vector Y
    z: 0.0  # Direction vector Z

```

### Response

* `distance` (`float32`): Euclidean distance to the first occupied cell. Returns **-1.0** if no obstacle is hit.
* `end_point` (`geometry_msgs/PointStamped`): The coordinates of the hit in the global map frame.

### Visualization

Every service call publishes a **red line marker** to the topic `distance_ray_marker` (Namespace:
`get_distance_to_obstacle`), visualizing the ray from origin to impact point in RViz.

### CLI Example

```bash
ros2 service call /get_distance_to_obstacle hector_worldmodel_msgs/srv/GetDistanceToObstacle \
"{point: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'back_lidar_cam_link'}, point: {x: 1.0, y: 0.0, z: 0.0}}}"
```
