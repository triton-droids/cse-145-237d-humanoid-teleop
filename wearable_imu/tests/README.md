# Test Workflow

This folder contains our verification checks.

Current tests cover:

- binary quaternion packet parsing
- quaternion spike rejection and smoothing
- neutral calibration profile creation
- toy planar IK behavior
- synthetic IMU orientation consistency
- seven-point lower-body skeleton aggregation
- lower-body geometry assumptions
- MuJoCo model loading and viewer helper outputs

We still need tests for:

- UDP receiver buffering and freshness checks
- neutral calibration repeatability
- thigh/shank heading alignment
- strap twist correction
- noisy synthetic IMU cases
