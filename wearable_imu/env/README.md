# Environment Workflow

This folder handles our environment setup.

We use Conda. Since shell activation has been unreliable on our machines, we
prefer running commands through the project environment:

```powershell
conda run -p .\.conda python -m pytest -q
```

The dependency definition lives in `environment.yml`.
