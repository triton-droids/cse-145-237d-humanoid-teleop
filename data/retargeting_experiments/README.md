# Retargeting Experiment Data

This folder stores small, curated experiment artifacts that verify the retargeting
pipeline ran successfully. It is intentionally not a dump of full training logs,
raw datasets, or downloaded third-party archives.

Keep experiment files here only when they are:

- small enough for normal GitHub review
- directly referenced by documentation or reports
- useful for validating file contracts, shapes, or downstream loaders
- derived from data that can be reproduced from documented source branches or scripts

Large raw captures, full OMOMO downloads, generated training logs, and repeated
checkpoints should stay out of the main branch. Use GitHub Releases or external
storage for those artifacts and link them from the relevant report.
