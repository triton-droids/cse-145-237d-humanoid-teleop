# Test Workflow

This folder contains verification checks.

Current tests cover:

- binary quaternion packet parsing
- quaternion spike rejection and smoothing
- neutral calibration profile creation
- toy planar IK behavior
- synthetic IMU orientation consistency
- lower-body geometry assumptions
- MuJoCo model loading and viewer helper outputs

Future tests should focus on:

- UDP receiver buffering and freshness checks
- neutral calibration repeatability
- thigh/shank heading alignment
- strap twist correction
- noisy synthetic IMU cases
