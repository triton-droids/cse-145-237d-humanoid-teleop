# Environment Workflow

This folder owns environment setup.

The project currently uses Conda. Because shell activation has been unreliable
on this machine, prefer running commands through the project environment:

```powershell
conda run --no-capture-output -n humanoid-sim python -m pytest -q
```

The dependency definition lives in `environment.yml`.
