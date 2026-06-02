# Evaluation Workflow

This folder is for measuring estimator behavior.

Possible checks:

- compare estimated joint angles against MuJoCo ground truth
- compare neutral-pose repeatability across calibration sessions
- measure drift over time
- test strap twist sensitivity
- test magnetometer disturbance sensitivity
- compare real captures against video or manually labeled events

Evaluation should report errors and confidence clearly enough that we know
whether a calibration or IK change actually helped.
