## What and why

<!-- What does this change, and what problem does it solve? Link the issue: Fixes #123 -->

## How it was tested

<!-- Tests added or run, and for physics changes the validation reference and tolerance. -->

## Checklist

- [ ] Builds in Debug and Release, and `sim_tests` passes
- [ ] Code formatted with the repository's `.clang-format` (CI pins clang-format 18.1.8)
- [ ] New files carry the SPDX header
- [ ] Everything is in English (code, comments, docs, commit messages)
- [ ] Commits are signed off (`git commit -s`, see `docs/CONTRIBUTING.md`)
- [ ] No real brands; any new content is original or freely licensed and recorded in `docs/CONTENT.md`
- [ ] Physics changes keep the simulation deterministic (no `-ffast-math`, no variable timestep)
