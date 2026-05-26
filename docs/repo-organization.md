# Repository Organization

This repository is organized around the full humanoid teleoperation pipeline rather than around individual experiments.

```text
cse-145-237d-humanoid-teleop/
├── wearable_imu/              # BNO085 + ESP32-S3 wearable sensing and IK
├── perception/depth_camera/   # RealSense + MediaPipe baseline
├── retargeting/               # Triton humanoid retargeting contract and Holosoma patch
├── simulation/                # Link to the dedicated Isaac Lab simulation repo
├── data/retargeting_samples/  # Small sample captures
├── docs/                      # Project documentation
├── reports/                   # Class deliverables
└── media/                     # Demo images and videos
```

## Public-Facing Content

The public-facing material should explain what the project is, how the pipeline works, who worked on it, and how a visitor can reproduce the main demos.

Keep these files clean enough to make the repository public:

- `README.md`
- `docs/project-overview-zh.md`
- `docs/replication-guide.md`
- `docs/system-architecture.md`
- `retargeting/README.md`
- `reports/`
- `media/`

## Simulation Repository

Isaac Lab training source and robot simulation assets are intentionally kept in a separate repository:

https://github.com/triton-droids/simulation

This avoids duplicating large meshes, USD assets, training scripts, checkpoints, and generated logs inside the course hub repository. Use [`../simulation/README.md`](../simulation/README.md) for the pinned commit reference.

## Internal Content

Internal planning notes can live in `docs/internal/` while the repository is private. Before making the repository public, review that folder for incomplete notes, private links, credentials, hardware network settings, or unsupported claims.

## Large Files

Large generated files should not live in the main branch:

- training checkpoints
- TensorBoard event files
- Hydra output folders
- raw long-duration logs
- downloaded third-party repositories
- zip archives

Use GitHub Releases or external storage and document the link.
