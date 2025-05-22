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
```

---

## 🔍 Get Distance to Obstacle Service

This package also provides a ROS 2 service, `/get_distance_to_obstacle`, that lets you query the octomap directly:

- **Service type:** `hector_worldmodel_msgs/srv/GetDistanceToObstacle`
- **Request:**
    - `point` (`geometry_msgs/PointStamped`)
        - `header.frame_id` – the sensor frame in which the ray origin is expressed
        - `point` – the direction vector (in sensor-frame coordinates) along which to cast the ray
- **Response:**
    - `distance` (`float32`) – distance (in meters) from the ray origin to the first occupied cell in the octomap
      - if the ray does not hit an occupied cell, the distance is `-1.0`
    - `end_point` (`geometry_msgs/PointStamped`) – the exact map-frame coordinates of that cell

### Implementation Details

1. **Transform & Raycast**
   The service callback transforms the input direction from the sensor frame into the octomap (map) frame, casts a ray through the octree, and computes the Euclidean distance to the first obstacle.

2. **Marker Visualization**
   Each call also publishes a `visualization_msgs/Marker` (`LINE_STRIP`) on `distance_ray_marker`, drawing a red line from the sensor origin to the impact point so you can instantly see your ray in RViz.

```bash
ros2 service call /get_distance_to_obstacle hector_worldmodel_msgs/srv/GetDistanceToObstacle \
"{point: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'back_lidar_cam_link'}, point: {x: 1.0, y: 0.0, z: 0.0}}}"
```
