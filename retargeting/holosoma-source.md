# Holosoma Source Branch

We keep the full Holosoma-based retargeting source on a separate branch:

https://github.com/triton-droids/cse-145-237d-humanoid-teleop/tree/retargeting_holosoma

Our main course-project branch intentionally keeps only the integration contract,
validation notes, and a portable patch under `retargeting/`. We don't vendor the
full `holosoma-main/` tree because it contains a complete third-party codebase,
robot assets, Docker/setup scripts, generated caches, and demo outputs.

## Recommended Workflow

Use a separate worktree when you need to inspect or continue the Holosoma code:

```bash
git fetch origin retargeting_holosoma
git worktree add ../cse-145-holosoma-retargeting origin/retargeting_holosoma
```

This keeps our course-project branch clean while still making the full source
available locally.

## What Belongs Here

We keep these files in this branch:

- `retargeting/README.md`
- `retargeting/ch_robot_contract.py`
- `retargeting/holosoma-ch-robot-converter-update.md`
- `retargeting/patches/0001-Add-ch_robot-support-to-retargeting-converter.patch`
- small contract examples under `retargeting/examples/`

We keep these out of this branch:

- local `holosoma-main/` copies
- `__pycache__/`, `.pyc`, and `.egg-info/`
- downloaded OMOMO archives
- generated retargeting outputs unless reduced to a small documented sample
- full robot mesh/source trees that already live in the Holosoma or simulation worktree

When the Holosoma implementation changes, we update the source branch first, then
refresh the patch and notes in this branch.
