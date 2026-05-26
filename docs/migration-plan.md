# Migration Plan

This repository consolidates work that was previously split across multiple branches and repositories.

## Source Mapping

| Source | Destination | Notes |
|---|---|---|
| `origin/inverse-kinematics` | `wearable_imu/` | Real-world IMU packet, calibration, IK, demo, and test code. |
| `origin/depthCamera` | `perception/depth_camera/` | RealSense D435 + MediaPipe pose-estimation baseline. |
| `origin/retargeting_data` | `data/retargeting_samples/` | Small sample JSONL captures. |
| `origin/retargeting_holosoma` | `retargeting/` | Project-specific CH robot retargeting notes and Holosoma converter patch only. |
| `triton-droids/simulation` | `simulation/README.md` link only | Current Isaac Lab humanoid simulation code stays in the dedicated simulation repository. |
| `origin/simulation` | partial / review needed | Contains Isaac Lab and MuJoCo material plus generated artifacts. Import only reviewed source files and demo media. |

## Next Migration Steps

1. Decide where to host large checkpoints and videos.
2. Add final report, final presentation, and demo links.
3. Review `docs/internal/` before making the repository public.
4. Update `simulation/README.md` when the dedicated simulation repository reaches a new stable commit.
5. Update `retargeting/patches/` when the Holosoma CH robot adapter changes.
