# Artifact Policy

We keep the main branch clean as our project source and documentation hub. Generated or bulky artifacts get linked, not committed.

## Do Not Commit

- `logs/`
- `outputs/`
- TensorBoard event files
- generated Hydra runs
- repeated training checkpoints
- raw long-duration capture logs
- downloaded third-party source trees
- zip archives
- credentials, Wi-Fi secrets, tokens, or private machine paths

## Acceptable To Commit

- small sample captures needed for tests or demos
- small curated experiment outputs that document a tested contract
- source code
- setup documentation
- small images used by reports
- final PDFs, if file size is reasonable
- short demo media, if GitHub handles it cleanly

## Recommended Hosting

We use GitHub Releases or external storage for:

- trained policy checkpoints
- full demo videos
- large raw captures
- full experiment logs

We then document the artifact link in `reports/` or `docs/`.
