#!/usr/bin/env python3
import bisect
import sys

# Usage:
#   python3 map_keyframes_to_frames.py times.txt KeyFrameTrajectory_unscaled.tum kf_to_frame.txt

times_path = sys.argv[1]
kf_tum_path = sys.argv[2]
out_path = sys.argv[3]

# Frame times: one float per line (seconds)
frame_times = []
with open(times_path, "r") as f:
    for line in f:
        line = line.strip()
        if line:
            frame_times.append(float(line))

def nearest_frame_index(t):
    i = bisect.bisect_left(frame_times, t)
    if i == 0:
        return 0
    if i >= len(frame_times):
        return len(frame_times) - 1
    before = frame_times[i - 1]
    after = frame_times[i]
    return i - 1 if abs(t - before) <= abs(t - after) else i

with open(out_path, "w") as out, open(kf_tum_path, "r") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        # TUM: timestamp tx ty tz qx qy qz qw
        t = float(parts[0])
        fi = nearest_frame_index(t)
        out.write(f"{fi} {t:.9f}\n")

print("Wrote:", out_path)
