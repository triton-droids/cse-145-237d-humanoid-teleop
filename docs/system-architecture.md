# System Architecture

The project is split into five subsystems.

```text
1. Wearable sensing
   BNO085 IMUs on ESP32-S3 nodes stream orientation packets over UDP.

2. Pose estimation
   Packets are parsed, filtered, smoothed, calibrated, and converted into lower-body segment poses.

3. Retargeting
   Human lower-body pose is mapped to the Triton humanoid kinematic model using the `ch_robot` joint contract.

4. Simulation and training
   Isaac Lab trains or evaluates policies that follow locomotion or reference-motion targets.

5. Deployment
   The trained policy and retargeted commands are prepared for Jetson and robot-side testing.
```

## Baselines

The depth-camera baseline uses an Intel RealSense D435 and MediaPipe to produce depth-backed body landmarks. This gives the team a non-wearable comparison path while the IMU pipeline is developed.

## Coordinate Contracts

The preferred downstream contract is robot-frame lower-body pose data. Sensor packets should retain timestamps, sensor identity, segment identity, quaternion order, and validity information so the receiver can reject stale or unstable data.

The retargeting output contract is documented in [`../retargeting/README.md`](../retargeting/README.md). For `ch_robot`, compact retargeted `qpos` has 17 dimensions: 7 floating-base values plus 10 robot joint positions.

## Simulation

Isaac Lab is the main policy-training environment and is maintained in the dedicated `triton-droids/simulation` repository. MuJoCo remains useful for synthetic IMU validation and sim-to-sim checks, but it is not the main training target.
