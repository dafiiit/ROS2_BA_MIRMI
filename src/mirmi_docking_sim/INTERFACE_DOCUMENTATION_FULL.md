# Interface Dokumentation (Full)
> Beinhaltet Nodes, Launch-Konfigurationen, Hardware (SDF) und Bridge.

## 1. Gestartete Nodes (Launch)
Hier sind Nodes definiert, die in Launchfiles gestartet werden (oft externe Pakete wie `apriltag_ros`).
Besonders wichtig sind die **Remappings**, die zeigen, welche Topics tatsächlich verbunden sind.

### `?` (Pkg: `ros_gz_bridge`)
- **Executable:** `parameter_bridge`
- **Definiert in:** `simulation.launch.py`

### `camera_info_sync_node` (Pkg: `mirmi_docking_sim`)
- **Executable:** `camera_info_sync`
- **Definiert in:** `simulation.launch.py`

### `apriltag_detector` (Pkg: `apriltag_ros`)
- **Executable:** `apriltag_node`
- **Definiert in:** `simulation.launch.py`
- **Parameter:**
  - `family: 36h11`
  - `size: 0.40`
  - `max_hamming: 0`
  - `image_transport: raw`
  - `publish_tag_detections_image: True`
  - `publish_tf: True`
  - `z_aligned: False`
  - `z_up: False`

### `depth_to_scan_node` (Pkg: `depthimage_to_laserscan`)
- **Executable:** `depthimage_to_laserscan_node`
- **Definiert in:** `simulation.launch.py`
- **Parameter:**
  - `range_min: 0.1`
  - `range_max: 100.0`
  - `scan_height: 10, # Nimmt 10 Pixelzeilen aus der Bildmitte`
  - `output_frame: robot/chassis/camera_sensor' # Selber Frame wie die Kameras`

### `static_tag_1_publisher` (Pkg: `tf2_ros`)
- **Executable:** `static_transform_publisher`
- **Definiert in:** `simulation.launch.py`

### `static_tag_2_publisher` (Pkg: `tf2_ros`)
- **Executable:** `static_transform_publisher`
- **Definiert in:** `simulation.launch.py`

### `static_tag_3_publisher` (Pkg: `tf2_ros`)
- **Executable:** `static_transform_publisher`
- **Definiert in:** `simulation.launch.py`

### `static_tag_4_publisher` (Pkg: `tf2_ros`)
- **Executable:** `static_transform_publisher`
- **Definiert in:** `simulation.launch.py`

### `apriltag_visualizer` (Pkg: `mirmi_docking_sim`)
- **Executable:** `apriltag_visualizer`
- **Definiert in:** `simulation.launch.py`

### `docking_controller` (Pkg: `mirmi_docking_sim`)
- **Executable:** `docking_controller`
- **Definiert in:** `simulation.launch.py`
- **Parameter:**
  - `use_ground_truth: LaunchConfiguration('use_ground_truth')`

### `odom_to_tf_publisher` (Pkg: `mirmi_docking_sim`)
- **Executable:** `odom_to_tf`
- **Definiert in:** `simulation.launch.py`

### `static_camera_publisher` (Pkg: `tf2_ros`)
- **Executable:** `static_transform_publisher`
- **Definiert in:** `simulation.launch.py`

### `automated_test_runner` (Pkg: `mirmi_docking_sim`)
- **Executable:** `automated_test_runner`
- **Definiert in:** `automated_test.launch.py`

---

## 2. Simulation Interfaces (SDF/Hardware)
Definiert die Sensoren und Topics, die Gazebo **intern** bereitstellt (Quelle der Daten).

### Model: `model.sdf`
| Sensor Name | Typ | Gazebo Topic (Raw) |
|---|---|---|
| `camera_sensor` | `camera` | `/model/robot/camera` |
| `depth_sensor` | `depth_camera` | `/model/robot/depth_camera` |

---

## 3. Eigene Python Nodes
### Node: `DockingTestRunner`
**Subs:** `/odom`, `/docking_controller/state`

**Pubs:** `/cmd_vel`, `/initialpose`


### Node: `FakeCameraInfoPublisher`
**Subs:** `/camera/image_raw`

**Pubs:** `/camera/camera_info`


### Node: `OdomToTFPublisher`
**Subs:** `/odom`


### Node: `AprilTagVisualizer`
**Subs:** `/camera/image_raw`, `/detections`

**Pubs:** `/camera/tag_detections_image`


### Node: `DockingController`
**Subs:** `/model/robot/pose`, `/odom`, `/detections`, `/initialpose`

**Pubs:** `/docking_controller/state`, `/cmd_vel`


---

## 4. Gazebo Bridge Mapping
| ROS Topic | Gazebo Topic | Richtung |
|---|---|---|
| `/cmd_vel` | `/model/robot/cmd_vel` | <- |
| `/odom` | `/model/robot/odometry` | -> |
| `/joint_states` | `/world/docking_world/model/robot/joint_state` | -> |
| `/camera/image_raw` | `/model/robot/camera` | -> |
| `/depth/image_raw` | `/model/robot/depth_camera/depth` | -> |
| `/depth/camera_info` | `/model/robot/depth_camera/depth/camera_info` | -> |
| `/model/robot/pose` | `/model/robot/pose` | -> |
| `/model/robot/set_pose` | `/model/robot/set_pose` | <- |