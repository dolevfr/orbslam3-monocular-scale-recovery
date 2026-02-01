```markdown
# ORB-SLAM3 Monocular Scale Recovery with Thread-Safe Map Access

## Overview
This repository documents and implements a complete pipeline for **metric scale recovery in monocular SLAM** using ORB-SLAM3.  
Monocular SLAM produces a geometrically consistent but **unitless** map. The goal of this project was to recover a **metric scale factor** using known real-world measurements, while **safely accessing map data in a multi-threaded SLAM system**.

The project was carried out on the **KITTI dataset**, using a standard **lane width reference of 3.7 meters**, and resulted in a recovered scale factor of approximately:

```

s ≈ 12.81

```

The final outputs are:
- a **metric-scaled camera trajectory**
- a **metric-scaled 3D point cloud**

---

## Problem Statement

### Monocular Scale Ambiguity
In monocular SLAM, depth and scale are not observable from vision alone. ORB-SLAM3 therefore reconstructs the scene up to an unknown scale factor `s`:

```

P_metric = s · P_slam

```

Recovering `s` requires introducing **external metric information**, such as a known physical distance visible in the image.

### Thread Safety in ORB-SLAM3
ORB-SLAM3 is a **multi-threaded system** consisting of:
- **Tracking**
- **Local Mapping**
- **Loop Closing**
- **Viewer (Pangolin)**

MapPoints and KeyFrames can be **created, optimized, replaced, or erased** while the system is running.  
Naively accessing map data during tracking can lead to:
- invalid memory access
- inconsistent reads
- deadlocks caused by incorrect mutex ordering

A major part of this project was identifying **how to safely access 3D map data** without interfering with the SLAM backend.

---

## Dataset Handling: KITTI Raw vs Odometry

### The Issue
The ORB-SLAM3 `mono_kitti` example expects a **KITTI Odometry-style layout**:

```

<sequence>/
image_0/000000.png
times.txt

```

However, **KITTI Raw** provides:
```

image_00/data/0000000000.png
image_00/timestamps.txt   (date strings)

```

This mismatch caused the system to hang or fail silently.

### The Solution: Wrapper Folder
To avoid modifying ORB-SLAM3 loaders, an **odometry-like wrapper directory** was created:

```

kitti_wrap_drive_0001/
image_0/          # images (10-digit filenames preserved)
times.txt         # float timestamps in seconds (relative)

```

This wrapper allowed `mono_kitti` to run correctly without changing upstream code.

---

## Camera Calibration
Rectified intrinsics for KITTI `image_00` were used, extracted from `P_rect_00`:

- fx = 721.5377  
- fy = 721.5377  
- cx = 609.5593  
- cy = 172.8540  
- image size = 1242 × 375  

These values were explicitly set in `KITTI_image00.yaml`.

---

## Reference Measurement

### Chosen Metric Reference
The project used the **standard highway lane width**:

```

D_metric = 3.7 meters

```

Two pixels corresponding to the left and right lane boundaries were manually selected in a chosen frame and stored in:

```

lane_clicks.txt

```

---

## Thread-Safe Map Access

### Relevant Mutex
The primary mutex protecting map structure in ORB-SLAM3 is:

```

Map::mMutexMapUpdate

````

### Deadlock Risk
- `Map` has its own mutex
- each `MapPoint` has an internal mutex
- locking both in the wrong order can deadlock the system

### Safe Access Strategy Used
1. **Lock `mMutexMapUpdate` briefly**
2. Snapshot and validate `MapPoint*` pointers (`!isBad()`)
3. **Release the map lock**
4. Access `MapPoint::GetWorldPos()` afterwards

This avoids holding the map mutex while invoking MapPoint methods that acquire their own locks.

---

## Code Modifications (Summary)

### Modified C++ Files
- **`src/Tracking.cc`**
  - Added logic to associate clicked pixels with nearest tracked keypoints
  - Snapshots MapPoint pointers under the map mutex
  - (Online scaling/logging exists but is disabled; offline is final)

- **`include/System.h`**
  - Added a minimal Atlas getter:
    ```
    Atlas* GetAtlasPointer() { return mpAtlas; }
    ```

- **`Examples/Monocular/mono_kitti.cc`**
  - Exports unscaled map to `map_unscaled.ply` after shutdown
  - Saves keyframe trajectory in TUM format

---

## Pipeline Flow

### 1) Build
````

cd ORB_SLAM3
./build.sh

```

### 2) Run SLAM (exports unscaled trajectory + map)
```

cd ORB_SLAM3
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI_image00.yaml /kitti_wrap_drive_0001

```

Outputs:
- `KeyFrameTrajectory_unscaled.tum`
- `map_unscaled.ply`

---

## Offline Scale Recovery

### Scale Computation
Using the exported map and trajectory, the scale factor is computed offline:

```

D_slam = ||P2 - P1||
s = D_metric / D_slam

```

With:
- `D_metric = 3.7 m`
- measured `D_slam ≈ 0.2889`

Result:
```

s ≈ 12.806

```

### Scripts Used
- `compute_scale_offline.py`  
  Finds corresponding MapPoints via reprojection and computes scale.

- `apply_scale.py`  
  Applies scale to:
  - all MapPoints → `map_scaled.ply`
  - camera translations → `KeyFrameTrajectory_scaled.tum`

- `verify_scaled_distance.py`  
  Confirms that the scaled distance between the two points is ≈ 3.7 m.

---

## Results

- Final recovered scale: **~12.81**
- Scaled map correctly reproduces lane width
- Trajectory and point cloud are metric-consistent
- No deadlocks or crashes during execution
- Offline approach ensures determinism and safety

---

## Notes and Observations

- Pangolin/OpenGL can block on some systems (especially over SSH).  
  Software rendering can be forced via:
```

export LIBGL_ALWAYS_SOFTWARE=1

```

- Different frames or reference points yield slightly different scales due to triangulation noise. Averaging multiple measurements improves robustness.

- Offline scaling was chosen as the **final pipeline** to avoid runtime interference with SLAM threads.

---

## Author
Dolev Freund  
MSc Electrical Engineering (Robotics)  
GitHub: https://github.com/dolevfr
```
