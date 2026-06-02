# Environment Workflow

This folder owns environment setup. The project targets **Python 3.11**.

The project uses Conda. Because shell activation has been unreliable on some
machines, prefer running commands through the project environment, e.g.:

```bash
conda run -n humanoid-sim python -m pytest -q
```

The dependency definition lives in `environment.yml`.

Actual runtime dependencies imported by the wearable_imu code are:

- `numpy`, `scipy` — quaternion/rotation math (`scipy.spatial.transform`)
- `matplotlib` — live 3D skeleton views and plots
- `mujoco` — synthetic-IMU viewer / regression harness
- `pytest` — test suite

`environment.yml` currently also pins `pinocchio`, `pandas`, `pydantic`, `tqdm`,
`ahrs`, and `transforms3d`, none of which are imported by the current code; they
can be dropped when the env is next regenerated. `tkinter` is used by the GUI
demos and ships with Python (not a pip package).
