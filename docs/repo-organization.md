# Repository Organization

We organized this repository around our full humanoid teleoperation pipeline rather than around individual experiments.

```text
cse-145-237d-humanoid-teleop/
├── wearable_imu/              # BNO085 + ESP32-S3 wearable sensing and IK
├── perception/depth_camera/   # RealSense + MediaPipe baseline
├── retargeting/               # Triton humanoid retargeting contract and Holosoma patch
├── simulation/                # Link to the dedicated Isaac Lab simulation repo
├── data/retargeting_samples/  # Small sample captures
├── data/retargeting_experiments/ # Curated small validation outputs
├── docs/                      # Project documentation
├── reports/                   # Class deliverables
└── media/                     # Demo images and videos
```

## Public-Facing Content

Our public-facing material explains what the project is, how the pipeline works, who worked on it, and how a visitor can reproduce the main demos.

We keep these files clean enough to make the repository public:

- `README.md`
- `docs/project-overview-zh.md`
- `docs/replication-guide.md`
- `docs/system-architecture.md`
- `retargeting/README.md`
- `data/retargeting_experiments/README.md`
- `reports/`
- `media/`

## Simulation Repository

We intentionally keep Isaac Lab training source and robot simulation assets in a separate repository:

https://github.com/triton-droids/simulation

This avoids duplicating large meshes, USD assets, training scripts, checkpoints, and generated logs inside the course hub repository. Use [`../simulation/README.md`](../simulation/README.md) for the pinned commit reference.

## Wearable IMU Source

We originally developed the wearable IMU and inverse-kinematics code on the `inverse-kinematics` branch:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/inverse-kinematics

In this integrated branch, that source is organized under [`../wearable_imu/`](../wearable_imu/). Use [`../wearable_imu/source-branch.md`](../wearable_imu/source-branch.md) for the branch-to-folder mapping and sync policy.

## Holosoma Retargeting Source

We intentionally keep the full Holosoma integration source on the `retargeting_holosoma` branch:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/retargeting_holosoma

We keep only the Triton humanoid retargeting contract, notes, and portable patch under [`../retargeting/`](../retargeting/). Local `holosoma-main/` copies, generated converter outputs, Python caches, downloaded datasets, and full robot asset trees should stay out of this branch. Use [`../retargeting/holosoma-source.md`](../retargeting/holosoma-source.md) as the entry point for working with the Holosoma branch.

## Internal Content

We keep internal planning notes in `docs/internal/` while the repository is private. Before making the repository public, review that folder for incomplete notes, private links, credentials, hardware network settings, or unsupported claims.

## Large Files

We keep large generated files out of the main branch:

- training checkpoints
- TensorBoard event files
- Hydra output folders
- raw long-duration logs
- downloaded third-party repositories
- zip archives

Use GitHub Releases or external storage and document the link.
