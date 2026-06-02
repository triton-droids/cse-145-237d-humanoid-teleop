# Evaluation Workflow

This folder is where we measure estimator behavior.

Possible checks:

- compare estimated joint angles against MuJoCo ground truth
- compare neutral-pose repeatability across calibration sessions
- measure drift over time
- test strap twist sensitivity
- test magnetometer disturbance sensitivity
- compare real captures against video or manually labeled events

We report errors and confidence clearly enough that we can tell whether a
calibration or IK change actually helped.
