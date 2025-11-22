
# SurfaceGuard: Multimodal 3D Defect Mapping 🏗️

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![PCL](https://img.shields.io/badge/Library-PCL-orange?style=for-the-badge)

**SurfaceGuard** is a hybrid perception stack designed for autonomous construction robots. It processes RGB-D sensor data to identify, localize, and measure surface defects (cracks, holes, uneven finishing) on vertical structures like drywall and concrete.

Unlike standard 2D detection pipelines, this project leverages **Geometric Scene Understanding** (C++/PCL) to physically locate defects in 3D space, enabling precise robotic arm actuation for repair tasks.

---

## 🚀 Key Features

* **Geometric Scene Understanding (C++17):** Implemented a high-performance **PCL** pipeline using **Voxel Grid** downsampling and **RANSAC** plane segmentation to mathematically isolate wall surfaces.
* **Hybrid Perception Architecture:** Designed to fuse semantic data (Deep Learning segmentation masks) with spatial data (Depth clouds) for context-aware defect mapping.
* **Actionable 3D Data:** Algorithms output 6-DOF centroids and surface area estimates for each defect, not just 2D bounding boxes.
* **Production-Grade ROS 2:** Structured with custom interfaces (`.msg`), lifecycle-ready nodes, and centralized parameter configuration (`.yaml`).

---

## 🛠️ Tech Stack

* **Middleware:** ROS 2 Humble (DDS)
* **Languages:** C++17 (Core Geometry Logic), Python 3.x (Deep Learning Wrapper)
* **Libraries:** PCL (Point Cloud Library), OpenCV, PyTorch, NumPy
* **Tools:** RViz2, Colcon, ROS Bag (Data Replay)

---

## 📂 Project Structure

```text
surface_guard_ws/
├── surface_guard_bringup/      # Launch files & RViz configurations
├── surface_guard_interfaces/   # Custom ROS 2 Message definitions (DefectArray.msg)
└── surface_guard_perception/   # Core Logic
    ├── src/wall_processor.cpp  # C++ Node (PCL, RANSAC, Clustering)
    ├── scripts/crack_detector.py # Python Node (Inference Wrapper)
    └── config/params.yaml      # Tunable thresholds
````

-----

## ⚡ Installation & Build

**Prerequisites:** Ubuntu 22.04, ROS 2 Humble

1.  **Clone the repository:**

    ```bash
    mkdir -p ~/surface_guard_ws/src
    cd ~/surface_guard_ws/src
    git clone [https://github.com/YOUR_USERNAME/SurfaceGuard.git](https://github.com/YOUR_USERNAME/SurfaceGuard.git) .
    ```

2.  **Install Dependencies:**

    ```bash
    cd ~/surface_guard_ws
    rosdep install --from-paths src --ignore-src -r -y
    sudo apt install ros-humble-pcl-conversions ros-humble-pcl-ros
    ```

3.  **Build the Workspace:**

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

-----

## 🏃 Usage (No Hardware Required)

This system is designed to run on **Intel RealSense datasets** (ROS Bags). You do not need physical hardware to verify the algorithms.

### 1\. Launch the Perception Stack

This command launches the C++ geometry node, the Python detector, and a pre-configured RViz visualization.

```bash
ros2 launch surface_guard_bringup system.launch.py
```

### 2\. Play Data (In a new terminal)

Feed the system with recorded depth data (e.g., from Intel's sample datasets or Gazebo simulation).

```bash
ros2 bag play path/to/your_wall_data.mcap --loop
```

*Note: Ensure your bag file publishes to `/camera/depth/color/points`. If you only have raw depth images, run `depth_image_proc` to convert them to points first.*

-----

## 🧠 Algorithmic Pipeline

1.  **Preprocessing:** Raw PointCloud2 data is downsampled (`leaf_size: 0.02m`) to ensure 30Hz real-time performance.
2.  **Plane Segmentation:** **RANSAC** (Random Sample Consensus) iteratively fits a plane model to the wall.
      * *Inliers* = The Wall (Safe Zone).
      * *Outliers* = Potential Defects/Obstacles.
3.  **Outlier Extraction:** Points falling outside the distance threshold (`0.02m`) are extracted.
4.  **Clustering:** **Euclidean Cluster Extraction** groups the noisy outlier points into distinct object instances.
5.  **Reporting:** Centroids are calculated, and a `DefectArray` message is published for the robot controller.

-----

## 🔮 Future Improvements

  * Implement **Kalman Filter** tracking to associate defects across multiple frames.
  * Integrate **YOLOv8-Seg** for semantic classification of specific defect types (e.g., distinguishing "screw hole" from "crack").
  * Add a **Behavior Tree** to trigger robotic arm motion planning when a defect is confirmed.

-----

## 👤 Author

**[Chaitanya Roy]**


<!-- end list -->

