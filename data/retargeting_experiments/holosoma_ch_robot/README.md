# Holosoma ch_robot Validation Sample

This folder has a small converted motion artifact from our Holosoma `ch_robot`
retargeting validation run.

## File

| Path | Purpose |
|---|---|
| `sub10_largebox_049_clip120_mj_fps50.npz` | Converted IsaacLab/RL tracking-format output from a 120-frame OMOMO clip retargeted to `ch_robot`. |

## Source

We keep the full Holosoma integration source on:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/retargeting_holosoma

We copied this sample from the simulation data path:

```text
source/tritonhumanoid/tritonhumanoid/data/motions/sub10_largebox_049_clip120_mj_fps50.npz
```

We intentionally don't commit the larger source archive `OMOMO_new.zip`.

## Verified Shape Contract

```text
fps: (1,)
joint_pos: (199, 17)
joint_vel: (199, 16)
body_pos_w: (199, 28, 3)
body_quat_w: (199, 28, 4)
body_lin_vel_w: (199, 28, 3)
body_ang_vel_w: (199, 28, 3)
joint_names: (10,)
body_names: (28,)
```

For `ch_robot`, `joint_pos` has width 17 because it contains 7 floating-base
values plus 10 robot joint positions. `joint_vel` has width 16 because MuJoCo
free-joint velocity is 6D.
