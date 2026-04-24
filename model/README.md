# Body Model Workflow

This folder contains shared body geometry and dimension assumptions.

It should define:

- body segment names
- segment coordinate frames
- default thigh, shank, foot, and pelvis dimensions
- default IMU placement assumptions
- anthropometric defaults from user height or manual measurements

Important boundary: quaternions alone do not determine bone lengths. Segment
lengths should come from manual measurement, user height ratios, or external
constraints.
