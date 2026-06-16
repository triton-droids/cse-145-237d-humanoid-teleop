# Retargeting Experiment Data

We store small, curated experiment artifacts here that verify our retargeting
pipeline ran correctly. This is intentionally not a dump of full training logs,
raw datasets, or downloaded third-party archives.

We only keep experiment files here when they are:

- small enough for normal GitHub review
- directly referenced by documentation or reports
- useful for validating file contracts, shapes, or downstream loaders
- derived from data that can be reproduced from documented source branches or scripts

We keep large raw captures, full OMOMO downloads, generated training logs, and repeated
checkpoints out of the main branch. We use GitHub Releases or external
storage for those and link them from the relevant report.
