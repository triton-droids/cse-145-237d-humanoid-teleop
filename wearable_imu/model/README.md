# Body Model Workflow

This folder contains shared body geometry and dimension assumptions.

It should define:

- body segment names
- segment coordinate frames
- default thigh, shank, foot, and pelvis dimensions
- default IMU placement assumptions
- anthropometric defaults from user height or manual measurements

Default wearable placement:

- pelvis: front center, around belt line
- thigh: lateral/outside thigh, around mid-thigh
- shank: lateral/outside lower leg, around mid-shank
- foot: top of foot, around midfoot/laces

Default segment axes:

- `+X` forward
- `+Y` left
- `+Z` up

Important boundary: quaternions alone do not determine bone lengths. Segment
lengths should come from manual measurement, user height ratios, or external
constraints.
