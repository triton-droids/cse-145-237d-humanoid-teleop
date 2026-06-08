# Environment Workflow

This folder owns environment setup. The project targets **Python 3.11**.

For the full, platform-by-platform setup guide (conda and raw venv, on
Linux/macOS/Windows), see
[Environment Setup](../README.md#environment-setup) in the main README. This
file is the short reference.

## Dependencies

Runtime dependencies are `numpy`, `scipy`, `matplotlib`, `mujoco`, `pyzmq`,
and `pytest` (plus `tkinter`, which ships with Python for the GUI demos). They
are declared in two equivalent places:

- [`environment.yml`](environment.yml) — for `conda env create`
- [`../requirements.txt`](../requirements.txt) — for `pip install`

Keep the two in sync when adding or removing a dependency.

## Conda (recommended)

Create the `humanoid-sim` environment from the spec:

```bash
cd /path/to/cse-145-237d-humanoid-teleop/wearable_imu
conda env create -f env/environment.yml
conda activate humanoid-sim
```

Replace `/path/to/cse-145-237d-humanoid-teleop` with the directory where you
cloned the repository. If the environment already exists, update it with:

```bash
conda env update -n humanoid-sim -f env/environment.yml --prune
```

Or build it manually with the `conda-forge` channel (avoids the default-channel
Terms-of-Service prompt) and install via pip:

```bash
conda create -y -n humanoid-sim -c conda-forge --override-channels python=3.11 pip
conda run -n humanoid-sim pip install -r requirements.txt
```

Because local shell activation has been unreliable on some machines, prefer
running commands through `conda run`:

```bash
conda run --no-capture-output -n humanoid-sim python -m pytest -q
```

## Raw venv (no conda)

```bash
python3.11 -m venv .venv
source .venv/bin/activate        # Windows: .\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements.txt
```
