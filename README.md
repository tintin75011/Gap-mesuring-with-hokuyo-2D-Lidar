# ScanFilter Package

## Overview
This ROS2 package provides filtering and clustering analysis tools for 2D LiDAR scan data. It consists of two main nodes:

1. **scan_filter_node**: Filters LiDAR scans based on a Region of Interest (ROI)
2. **scan_analysis_node**: Accumulates filtered scans and performs DBSCAN clustering to detect objects and measure distances between them

## Features

### scan_filter_node
- Real-time ROI (Region of Interest) filtering
- Converts polar coordinates (range, angle) to Cartesian (x, y)
- Publishes filtered scans to `/filteredScan`
- Configurable rectangular ROI bounds

### scan_analysis_node
- **Accumulation mode**: Collects scan data over a configurable time period (default: 10 seconds)
- **DBSCAN clustering**: Detects and extracts the 2 largest point clusters
- **Distance measurement**: Calculates minimum distance between the two detected clusters
- **RViz visualization**: 
  - Colored point clouds for each cluster
  - Distance line and label between clusters
  - Cluster labels with point counts

## Supported Environment
- ROS2 (tested on Humble)
- Python 3
- Dependencies: numpy, scikit-learn

## License
`Apache License 2.0`

## Subscribed Topics

### scan_filter_node
- **/scan** (sensor_msgs::msg::LaserScan)  
  Raw LiDAR scanning data (typically from Hokuyo LiDAR)

### scan_analysis_node
- **/filteredScan** (sensor_msgs::msg::LaserScan)  
  Filtered LiDAR scanning data from scan_filter_node

## Published Topics

### scan_filter_node
- **/filteredScan** (sensor_msgs::msg::LaserScan)  
  ROI-filtered LiDAR scanning data

### scan_analysis_node
- **/clusters_markers** (visualization_msgs::msg::MarkerArray)  
  Visualization markers for detected clusters (colored points and labels)
- **/distance_markers** (visualization_msgs::msg::MarkerArray)  
  Visualization markers for distance measurement (line and text)

## Parameters

### scan_filter_node
- **x_min** (float, default: 0.0)  
  Minimum X coordinate of ROI in meters
- **x_max** (float, default: 0.30)  
  Maximum X coordinate of ROI in meters
- **y_min** (float, default: -0.050)  
  Minimum Y coordinate of ROI in meters
- **y_max** (float, default: 0.05)  
  Maximum Y coordinate of ROI in meters

### scan_analysis_node
- **accumulation_time** (float, default: 10.0)  
  Duration in seconds to accumulate scan data before processing
- **eps** (float, default: 0.006)  
  Maximum distance between two samples for DBSCAN clustering (in meters)
- **min_samples** (int, default: 50)  
  Minimum number of points required to form a cluster
- **marker_size** (float, default: 0.001)  
  Size of point markers in RViz (in meters)

## How to Build

1. **Create ROS2 workspace and obtain source code**
```bash
   cd <ROS2_workspace>/src
   git clone <your_repository_url> ScanFilter
```

2. **Install dependencies**
```bash
   cd <ROS2_workspace>
   rosdep update
   rosdep install -i --from-paths src/ScanFilter
   pip install numpy scikit-learn
```

3. **Build the package**
```bash
   cd <ROS2_workspace>
   colcon build --packages-select ScanFilter --symlink-install
```

4. **Source the workspace**
```bash
   source install/setup.bash
```

## Usage Examples

### Launch All Nodes Together

The easiest way to start the complete pipeline is using the launch file:

```bash
ros2 launch ScanFilter scan_analysis.launch.py
```

This will start:
- Static TF publisher (world → laser)
- scan_filter_node (ROI filtering)
- scan_analysis_node (clustering and distance measurement)
- RViz2 (visualization)

### Launch with Custom Parameters

```bash
# Modify ROI bounds
ros2 launch ScanFilter scan_analysis.launch.py \
    x_min:=0.1 \
    x_max:=0.4 \
    y_min:=-0.1 \
    y_max:=0.1

# Modify accumulation time and clustering parameters
ros2 launch ScanFilter scan_analysis.launch.py \
    accumulation_time:=5.0 \
    eps:=0.008 \
    min_samples:=100 \
    marker_size:=0.002

# Combine multiple parameters
ros2 launch ScanFilter scan_analysis.launch.py \
    x_max:=0.5 \
    accumulation_time:=15.0 \
    eps:=0.01
```

### Run Nodes Individually

If you need to run nodes separately:

```bash
# Terminal 1: Filter node
ros2 run ScanFilter scan_filter_node

# Terminal 2: Analysis node
ros2 run ScanFilter scan_analysis_node

# Terminal 3: RViz
ros2 run rviz2 rviz2
```

### Runtime Parameter Changes

You can modify parameters while nodes are running:

```bash
# Change accumulation time
ros2 param set /scan_analysis_node accumulation_time 15.0

# Change DBSCAN epsilon
ros2 param set /scan_analysis_node eps 0.008

# Change minimum samples for clustering
ros2 param set /scan_analysis_node min_samples 100

# Change ROI bounds
ros2 param set /scan_filter_node x_max 0.5
ros2 param set /scan_filter_node y_min -0.1
```

### View Current Parameters

```bash
# List all parameters for a node
ros2 param list /scan_analysis_node

# Get specific parameter value
ros2 param get /scan_analysis_node accumulation_time
```

## RViz Configuration

To visualize the results in RViz2:

1. **Add MarkerArray displays**:
   - Topic: `/clusters_markers` (shows colored point clusters)
   - Topic: `/distance_markers` (shows distance line and measurement)

2. **Set Fixed Frame**: `laser` or `world`

3. **Optional**: Add LaserScan display for `/filteredScan` to see raw filtered points

## Workflow

1. **Data Collection** (0-10 seconds):
   - LiDAR publishes raw scans to `/scan`
   - scan_filter_node filters points within ROI
   - scan_analysis_node accumulates filtered points

2. **Processing** (~0.5 seconds):
   - DBSCAN clustering on accumulated points
   - Extract 2 largest clusters
   - Calculate minimum distance between clusters
   - Generate visualization markers

3. **Visualization**:
   - Clusters displayed as colored points (Red: G0, Blue: G1)
   - Distance line connecting nearest points
   - Text labels showing cluster sizes and distance

4. **Repeat**: New accumulation cycle begins automatically

## Troubleshooting

### No clusters detected
- Check `eps` parameter (try increasing it: 0.008 - 0.01)
- Check `min_samples` parameter (try decreasing it: 30 - 40)
- Verify ROI contains your objects of interest
- Check `/filteredScan` topic for sufficient points

### Distance not calculated
- Ensure exactly 2 clusters are detected
- Check logs for error messages with `ros2 node logs /scan_analysis_node`
- Verify both clusters have sufficient points

### RViz warnings "queue is full"
- This is normal during processing phase
- Messages are dropped but visualization still works
- Consider reducing `marker_size` if performance is an issue

### Poor clustering results
- Adjust `accumulation_time` (longer = more stable, but less responsive)
- Fine-tune `eps` based on your object spacing
- Ensure objects are distinguishable in point cloud

## Node Information

### scan_filter_node
- **Type**: Standard ROS2 node
- **Processing**: Real-time filtering
- **Performance**: Minimal latency (<1ms typical)

### scan_analysis_node
- **Type**: Standard ROS2 node
- **Processing**: Batch processing (every 10 seconds by default)
- **Performance**: ~0.5s processing time for ~17,000 points

## Example Output

[INFO] [scan_analysis_node]: 🔵 Début de l'accumulation des données...

[INFO] [scan_analysis_node]: ⏳ Accumulation en cours... 1906 points (9.0s restantes)

[INFO] [scan_analysis_node]: ⏳ Accumulation en cours... 3646 points (7.9s restantes)

...

[INFO] [scan_analysis_node]: 🟢 Traitement de 17355 points accumulés...

[INFO] [scan_analysis_node]: 2 groupe(s) détecté(s) | G0: 9162 pts | G1: 8193 pts

[INFO] [scan_analysis_node]: ✅ Traitement terminé en 0.5124s

[INFO] [scan_analysis_node]: 📏 Distance minimale entre G0 et G1 : 0.0156 m


## Integration with Hokuyo LiDAR

This package is designed to work with the Hokuyo urg_node2 driver. Complete setup:

```bash
# Terminal 1: Start Hokuyo LiDAR driver
ros2 launch urg_node2 urg_node2.launch.py

# Terminal 2: Start filtering and analysis
ros2 launch ScanFilter scan_analysis.launch.py
```

Make sure your Hokuyo LiDAR is publishing to `/scan` topic.
