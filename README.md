# ORB-SLAM3 Monocular Scale Recovery with Thread-Safe Map Access

## Overview
This project extends ORB-SLAM3 to demonstrate metric scale recovery in monocular SLAM while respecting the system’s multi-threaded map architecture. Monocular SLAM produces a geometrically correct but unitless map; this work recovers a metric scale using user-selected image features with known real-world distances (e.g., standard lane width), evaluated on KITTI.

The end result is a **scaled trajectory** and **scaled point cloud**:
- `KeyFrameTrajectory_scaled.tum`
- `map_scaled.ply`

---

## What This Project Solves

### 1) Monocular scale ambiguity (unitless map)
Monocular SLAM cannot recover absolute scale. ORB-SLAM3 outputs a map and poses up to an unknown scale factor `s`:

```

P_metric = s · P_slam

```

### 2) Thread-safe map access in ORB-SLAM3
ORB-SLAM3 is multi-threaded. While Tracking runs, the Local Mapper and Loop Closer can modify / optimize / replace / erase MapPoints. Reading 3D map data incorrectly can cause:
- crashes (reading invalid pointers)
- inconsistent reads
- deadlocks (wrong lock ordering)

This project identifies the correct mutex and uses safe access patterns to avoid those issues.

### 3) KITTI Raw vs KITTI Odometry mismatch (the “wrap” problem)
The ORB-SLAM3 `mono_kitti` example typically expects KITTI **Odometry-style** layout:

```

<sequence>/
image_0/000000.png ...
times.txt

```

KITTI Raw is different:

```

.../image_00/data/0000000000.png ...
.../image_00/timestamps.txt   (date strings)

```

To run without modifying ORB-SLAM3 loaders, we created an **odometry-like wrapper** directory:
- `kitti_wrap_drive_0001/image_0/` (images)
- `kitti_wrap_drive_0001/times.txt` (float timestamps, relative seconds)

---

## System Architecture (How ORB-SLAM3 Runs)

ORB-SLAM3 runs multiple threads in parallel:

- **Tracking thread**
  - reads images
  - extracts ORB features
  - matches features
  - associates keypoints to MapPoints (`Frame::mvpMapPoints`)
  - creates KeyFrames occasionally

- **Local Mapping thread**
  - triangulates new MapPoints
  - runs local bundle adjustment
  - culls/merges/replaces points

- **Loop Closing thread**
  - detects loops
  - applies global pose-graph / optimization (may replace/cull points)

- **Viewer thread (Pangolin)**
  - visualizes map and camera poses (can hang if OpenGL issues)

The critical consequence:
> MapPoints can change while Tracking is running, so extracting 3D data must be synchronized.

---

## Thread Safety and Locking Strategy

### The lock that matters
Map-level updates are protected by:
- `Map::mMutexMapUpdate` (the main mutex we use to safely snapshot map data)

### Deadlock risk (why we were careful)
MapPoints also have their own internal mutexes. A common deadlock pattern is:
- Thread A: lock Map mutex → call MapPoint method (locks MP mutex)
- Thread B: lock MP mutex → tries to lock Map mutex
=> deadlock.

### Safe pattern used in this project
- Lock `Map::mMutexMapUpdate` briefly only to **snapshot/validate pointers**
- Release map lock
- Then call `MapPoint->GetWorldPos()` (which may take its own internal lock)

This avoids holding Map and MapPoint locks simultaneously in the wrong order.

---

## Project Flow (Top-to-Bottom)

### Step 0: Build
```

cd ORB_SLAM3
./build.sh

```

### Step 1: Run ORB-SLAM3 on the wrapper dataset
This repo includes the wrapper directory inside `ORB_SLAM3` so the run command uses an absolute path:

```

cd ORB_SLAM3
./Examples/Monocular/mono_kitti 
Vocabulary/ORBvoc.txt 
Examples/Monocular/KITTI_image00.yaml 
/kitti_wrap_drive_0001

```

Notes:
- If Pangolin/OpenGL hangs, you can test software rendering:
```

export LIBGL_ALWAYS_SOFTWARE=1

```

### Step 2: Export unscaled trajectory + map
After shutdown, the modified example exports:
- `KeyFrameTrajectory_unscaled.tum`
- `map_unscaled.ply`

These are in SLAM (unitless) scale.

### Step 3: Provide reference measurement (pixels + known metric distance)
You manually pick two pixels in a chosen frame where the real-world distance is known.

Examples of allowed reference distances:
- Standard lane width: **3.7 m**
- Dashed line length: **3.0 m**
- Sedan wheelbase: **2.7 m**

These clicks are stored in:
- `lane_clicks.txt`

### Step 4: Offline scale computation (robust + deterministic)
We compute the scale factor offline using exported map + trajectory:
- project MapPoints into the chosen camera/keyframe
- find the two MapPoints whose projections match the clicked pixels
- compute:
```

D_slam = ||P2 - P1||
s = D_metric / D_slam

````

### Step 5: Apply scale to trajectory and point cloud
Scale is applied to:
- all MapPoint coordinates in PLY
- translation component of poses in TUM trajectory
(Rotation is unchanged)

### Step 6: Verify
Finally, we verify that the measured distance between the matched scaled points is ≈ the known metric value (e.g., 3.7 m).

---

## Files and What Each One Does

### Core run / build
- `build.sh`  
Builds ORB-SLAM3 and examples.

- `build_ros.sh`  
ROS build helper (not required for this project’s core flow).

- `CMakeLists.txt`  
Build configuration (may include minor changes needed for added code).

### Inputs (your measurement + dataset wrapper)
- `kitti_wrap_drive_0001/`  
Dataset wrapper directory expected by `mono_kitti`.
- `image_0/` images
- `times.txt` float timestamps (relative seconds)

- `lane_clicks.txt`  
Your measurement input: frame id + two pixel coordinates + known metric distance.

### Intermediate mapping helpers (frame ↔ time ↔ keyframe)
These exist to connect a “clicked frame index” to the best matching KeyFrame:
- `frame_to_timestamp.txt`  
Maps frame index → timestamp (derived from `times.txt` or run logs).

- `keyframe_to_frame.txt`  
Maps KeyFrame id → closest frame index (so offline projection uses the correct pose).

- `map_keyframes_to_frames.py`  
Generates `keyframe_to_frame.txt` using timestamps and trajectory alignment logic.

### Exported outputs (from ORB-SLAM3)
- `KeyFrameTrajectory.txt` / `KeyFrameTrajectory_unscaled.tum`  
Unscaled keyframe trajectory (poses in SLAM units).

- `map_unscaled.ply`  
Unscaled point cloud exported from the map (SLAM units).

### Offline scale computation and scaling
- `compute_scale_offline.py`  
Main offline algorithm:
- loads `map_unscaled.ply`, `KeyFrameTrajectory_unscaled.tum`, `lane_clicks.txt`
- selects the best KeyFrame for the clicked frame (using mapping files)
- projects MapPoints into the camera
- finds the closest projected points to each click
- computes `D_slam` and scale `s`
- writes/updates logs as needed

- `apply_scale.py`  
Applies scale `s` to generate:
- `KeyFrameTrajectory_scaled.tum`
- `map_scaled.ply`

- `verify_scaled_distance.py`  
Verifies the result by measuring the 3D distance in the scaled map between the two matched points. Expected output ~3.7 m (or your chosen metric reference).

### Logs and final products
- `scale_log.txt`  
Logs computed scales (either from online logging or offline scripts), useful for debugging and repeatability.

- `KeyFrameTrajectory_scaled.tum`  
Final scaled trajectory (metric translations).

- `map_scaled.ply`  
Final scaled point cloud (metric coordinates).

### Code changes inside ORB-SLAM3
Modified files (high level):
- `src/Tracking.cc`
- added logic to associate clicked pixels → nearest tracked keypoints → MapPoints
- included safe snapshotting under map lock (originally for online logging)
- online scaling/logging can be disabled (offline is the final pipeline)

- `include/System.h`
- added a minimal Atlas getter used for exporting map data:
  ```
  Atlas* GetAtlasPointer() { return mpAtlas; }
  ```

- `Examples/Monocular/mono_kitti.cc`
- exports unscaled map (`map_unscaled.ply`) after shutdown
- saves keyframe trajectory to TUM format

## Running the Full Pipeline

### 1) Build
```

cd ORB_SLAM3
./build.sh

```

### 2) Run SLAM (exports unscaled trajectory + map)
```

cd ORB_SLAM3
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI_image00.yaml /kitti_wrap_drive_0001

```

### 3) Compute scale offline
Example (adjust to your script arguments):
```

python3 compute_scale_offline.py --frame 50 --width_m 3.7

```

### 4) Apply scale
```

python3 apply_scale.py --scale <SCALE_VALUE_FROM_STEP_3>

```

### 5) Verify
```

python3 verify_scaled_distance.py --frame 50

```

---

## Notes / Common Issues

### Pangolin/OpenGL hangs
If the viewer blocks (common on SSH / broken GL), try:
```

export LIBGL_ALWAYS_SOFTWARE=1

```

### KITTI raw filename formatting
The `mono_kitti` loader may expect 6-digit names (`000000.png`) while raw images are 10-digit (`0000000000.png`). The wrapper folder must match what the loader expects. This repo uses the wrapper format that successfully runs with the current loader setup.

---

## Author
Dolev Freund  
MSc Electrical Engineering (Robotics)  
GitHub: https://github.com/dolevfr
```
