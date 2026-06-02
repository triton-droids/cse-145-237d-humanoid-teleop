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

Default segment axes (body/world frame):

- `+X` forward
- `+Y` left
- `+Z` up

Default IMU sensor axes per segment, expressed in the body frame above. The
sensor frame is `+X` = device top, `+Y` = device left edge, `+Z` = out of the
chip face. The mounts are defined in `ik/imu_orientation.py`.

| Segment | Sensor `+X` (device top) | Sensor `+Y` (left edge) | Sensor `+Z` (out of face) |
|---|---|---|---|
| pelvis | up (`+Z`) | right (`-Y`) | forward (`+X`) |
| left thigh | up (`+Z`) | forward (`+X`) | left/outward (`+Y`) |
| left shank | up (`+Z`) | forward (`+X`) | left/outward (`+Y`) |
| left foot | forward (`+X`) | left (`+Y`) | up (`+Z`) |
| right thigh | up (`+Z`) | backward (`-X`) | right/outward (`-Y`) |
| right shank | up (`+Z`) | backward (`-X`) | right/outward (`-Y`) |
| right foot | forward (`+X`) | left (`+Y`) | up (`+Z`) |

General rule: sensor `+Z` always points **outward** (out of the chip face, away
from the body-mounting surface), and `+Y` completes a right-handed frame, which
is why the left and right legs mirror each other (`+Y` flips between forward and
backward). Sensor `+X` points **up the limb toward the hip** for the pelvis,
thighs, and shanks. The foot is the exception: it lies flat, so `+X` points
**forward toward the toes** and the left and right foot mounts are identical.

Important boundary: quaternions alone do not determine bone lengths. Segment
lengths should come from manual measurement, user height ratios, or external
constraints.
