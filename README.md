# ORB-SLAM3 Monocular Scale Recovery with Thread-Safe Map Access

## Overview

This project extends ORB-SLAM3 to demonstrate metric scale recovery in monocular SLAM while respecting the system’s multi-threaded map architecture. Monocular SLAM produces a geometrically correct but unitless map; this work recovers a metric scale using user-selected image features with known real-world distances, evaluated on the KITTI dataset.

## Objectives

* Run ORB-SLAM3 on a KITTI monocular sequence
* Identify image features with known real-world distances (e.g. lane width)
* Safely access 3D map points in a multi-threaded SLAM system
* Compute a metric scale factor
* Apply the scale to the camera trajectory and point cloud

## Key Challenges Addressed

### 1. Monocular Scale Ambiguity

ORB-SLAM3 does not recover metric scale when using a single camera.
A scale factor `s` must be computed such that:

```
P_metric = s · P_slam
```

### 2. Multi-Threaded Map Safety

ORB-SLAM3 runs Tracking, Local Mapping, and Loop Closing in parallel.
MapPoints may be optimized, replaced, or erased at any time.

This project demonstrates:

* correct identification of the relevant mutex (`Map::mMutexMapUpdate`)
* validation of MapPoints using `!isBad()`
* avoidance of deadlocks by respecting lock ordering
* safe snapshotting of map data without blocking the SLAM backend

## Dataset Layout

The repository includes a KITTI wrapper folder adapted to the layout expected by the `mono_kitti` example.

ORB_SLAM3/

* kitti_wrap_drive_0001/

  * image_0/

    * 0000000000.png
    * 0000000001.png
    * ...
  * times.txt

`image_0/` contains rectified KITTI images.
`times.txt` contains relative timestamps in seconds.

## Build Instructions

```
cd ORB_SLAM3
./build.sh
```

## Running the System

```
cd ORB_SLAM3
./Examples/Monocular/mono_kitti \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/KITTI_image00.yaml \
  /kitti_wrap_drive_0001
```

## Reference Feature Selection

Reference points are selected manually in images where real-world distances are known.
Examples used in this project include:

* Standard lane width: 3.7 meters
* Dashed highway line length: 3.0 meters
* Standard sedan wheelbase: 2.7 meters

Pixel coordinates and frame indices are stored in `lane_clicks.txt`.

## Scale Computation

Given two selected 3D SLAM points `P1` and `P2`:

```
D_slam = ||P2 - P1||
s = D_metric / D_slam
```

## Scaling the Map and Trajectory

Scaling is applied offline for determinism and safety.

* Map points: `P_scaled = s · P`
* Camera translations: `t_scaled = s · t`
* Rotations remain unchanged

Generated outputs:

* `map_unscaled.ply`
* `map_scaled.ply`
* `KeyFrameTrajectory_unscaled.tum`
* `KeyFrameTrajectory_scaled.tum`

## Thread Safety Strategy

* Map access is protected using `Map::mMutexMapUpdate`
* MapPoints are validated using `!isBad()`
* No Map mutex is held while calling MapPoint methods that acquire internal locks
* Offline processing avoids runtime race conditions

## Results

After scaling, measured distances in the reconstructed map closely match real-world values (e.g. lane width ≈ 3.7 m), validating correct scale recovery.

## Notes

* Online scale logging exists but is disabled by default
* Offline scaling was chosen for robustness and reproducibility
* The focus is correctness and safe map access rather than performance

## Author

Dolev Freund
MSc Electrical Engineering (Robotics)
GitHub: [https://github.com/dolevfr](https://github.com/dolevfr)
