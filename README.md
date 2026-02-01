```markdown
# ORB-SLAM3 Monocular Scale Recovery with Thread-Safe Map Access

## Overview

This project extends **ORB-SLAM3** to recover **metric scale** in a **monocular SLAM** pipeline while respecting the system’s **multi-threaded map architecture**.  
Monocular SLAM reconstructs geometry correctly but only **up to an unknown scale**. The goal of this work is to compute a scale factor using **user-selected image features with known real-world distances**, and to apply that scale safely to the reconstructed trajectory and map.

All experiments were performed on the **KITTI dataset**, using a wrapped KITTI Raw sequence to match the expected ORB-SLAM3 monocular interface.

---

## Problem Statement

Given two image points with a known real-world distance \( D_{\text{metric}} \), and their corresponding 3D map points reconstructed by ORB-SLAM3, compute a scale factor \( s \) such that:

\[
P_{\text{metric}} = s \cdot P_{\text{slam}}
\]

where \( P_{\text{slam}} \) are the original unitless SLAM coordinates.

The main challenges are:
1. **Monocular scale ambiguity**
2. **Thread-safe access to map data in a multi-threaded SLAM system**
3. **Dataset format mismatch (KITTI Raw vs KITTI Odometry)**

---

## Dataset Wrapping (KITTI Raw → Odometry-like)

The `mono_kitti` example in ORB-SLAM3 expects an **odometry-style layout**:

```

<sequence>/
image_0/000000.png ...
times.txt

```

KITTI Raw provides:
```

image_00/data/0000000000.png
image_00/timestamps.txt   (absolute timestamps)

```

To avoid modifying ORB-SLAM3 loaders, an **odometry-like wrapper folder** was created:

```

kitti_wrap_drive_0001/
├── image_0/        (symlinked images)
└── times.txt       (relative timestamps in seconds)

```

This wrapper allowed `mono_kitti` to run correctly without changes to the dataset parser.

---

## ORB-SLAM3 Threading Model

ORB-SLAM3 runs multiple threads concurrently:

- **Tracking**  
  Processes images, extracts features, associates keypoints with MapPoints.
- **Local Mapping**  
  Creates, optimizes, replaces, and deletes MapPoints.
- **Loop Closing**  
  Performs global pose graph optimization and map corrections.
- **Viewer (Pangolin)**  
  Visualizes the map and trajectory.

Because MapPoints may be **optimized, replaced, or erased at any time**, accessing 3D map data requires strict synchronization.

---

## Thread Safety and Mutex Handling

### Relevant Mutex
- `Map::mMutexMapUpdate`

### Deadlock Risk
Each `MapPoint` also owns an internal mutex.  
A naïve approach such as:

1. Lock Map mutex  
2. Call `MapPoint::GetWorldPos()` (locks MapPoint mutex)

can deadlock if another thread locks the MapPoint first and then tries to lock the Map.

### Safe Access Pattern Used
1. Lock `Map::mMutexMapUpdate`
2. **Snapshot MapPoint pointers and validate `!isBad()`**
3. Unlock the Map mutex
4. Read MapPoint world positions

This guarantees:
- No simultaneous Map + MapPoint lock ordering violations
- Safe, consistent reads without blocking backend threads

---

## Reference Measurement

The metric reference used in this project was:

- **Standard lane width:**  
  \[
  D_{\text{metric}} = 3.7 \text{ meters}
  \]

Two pixels corresponding to the left and right lane boundaries were manually selected in a chosen frame and stored in:

```

lane_clicks.txt

````

---

## Scale Computation

Given two corresponding 3D SLAM points \( P_1, P_2 \):

\[
D_{\text{slam}} = \| P_2 - P_1 \|
\]

\[
s = \frac{D_{\text{metric}}}{D_{\text{slam}}}
\]

### Measured Result

From the selected frame:

- \( D_{\text{slam}} \approx 0.288925499 \)
- \( D_{\text{metric}} = 3.7 \, \text{m} \)

\[
\boxed{s \approx 12.806069374}
\]

This scale factor was used for all subsequent scaling.

---

## Offline Scaling Strategy

Although online scale logging was implemented inside the Tracking thread, the **final pipeline performs scaling offline** for robustness and determinism.

### Exported (Unscaled)
- `KeyFrameTrajectory_unscaled.tum`
- `map_unscaled.ply`

### Scaling
- Map points:
\[
P_i' = s \cdot P_i
\]
- Camera translations:
\[
t' = s \cdot t
\]
- Rotations unchanged

### Final Outputs
- `KeyFrameTrajectory_scaled.tum`
- `map_scaled.ply`

---

## Verification

After scaling, the distance between the two reconstructed lane boundary points in `map_scaled.ply` was measured:

\[
D_{\text{measured}} \approx 3.7 \text{ meters}
\]

confirming correct scale recovery.

---

## Modified / Added Files

### C++ (ORB-SLAM3)
- `src/Tracking.cc`  
  - Pixel → keypoint → MapPoint association  
  - Safe snapshotting under `Map::mMutexMapUpdate`  
  - Optional online scale logging
- `include/System.h`  
  - Added minimal Atlas getter:
    ```
    Atlas* GetAtlasPointer() { return mpAtlas; }
    ```
- `Examples/Monocular/mono_kitti.cc`  
  - Export of unscaled map (`map_unscaled.ply`) after shutdown  
  - Save keyframe trajectory in TUM format

### Python (Offline Processing)
- `compute_scale_offline.py` – computes scale from clicks and map
- `apply_scale.py` – applies scale to trajectory and map
- `verify_scaled_distance.py` – verifies metric correctness
- `map_keyframes_to_frames.py` – keyframe ↔ frame association
- `frame_to_timestamp.txt`, `keyframe_to_frame.txt` – helper mappings

---

## Running the Full Pipeline

### 1) Build
````

cd ORB_SLAM3
./build.sh

```

### 2) Run SLAM
```

cd ORB_SLAM3
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI_image00.yaml /kitti_wrap_drive_0001

```

### 3) Compute Scale
```

python3 compute_scale_offline.py --frame 50 --width_m 3.7

```

### 4) Apply Scale
```

python3 apply_scale.py --scale 12.806069374

```

### 5) Verify
```

python3 verify_scaled_distance.py --frame 50

```

---

## Notes

- Pangolin/OpenGL issues may require:
```

export LIBGL_ALWAYS_SOFTWARE=1

```
- Online scale logging exists but is disabled; offline scaling is the final method.
- The focus of this project is **correctness, thread safety, and reproducibility**, not real-time performance.

---

## Author

Dolev Freund  
MSc Electrical Engineering (Robotics)  
GitHub: https://github.com/dolevfr
```
