# iPTAM maintenance workflow

Read `SUMMARY.md`, this file and `README.md` before editing.

## Working on the repository

Use the historical report to qualify tracking claims. Keep proposed ports or modern-device work separate from preserved code. Preserve PTAM, LibCVD and TooN license notices independently; no single replacement license applies to all components.

## Documentation checks

- Check claims against the named source files and recorded results. Separate source inspection,
  configuration generation, runtime execution and benchmark evidence.
- Keep README setup requirements available without following an advanced guide. Check Markdown
  headings, links and image references, including source files omitted by a sparse checkout.
- Inspect labels inside diagrams as well as captions. Never turn a schematic or generated input
  into an implied successful run.
- Keep original source, credits and license notices intact for documentation-only changes.
- Review and synchronize `SUMMARY.md` and `SKILL.md` before every commit. Record any unrun checks
  as limitations rather than implying that validation passed.

## Current evidence boundary

Project/camera/display source and historical report inspection only; no current Xcode build, iPhone deployment, tracking test or runtime measurement.

Unmaintained port, old SDK/signing/API requirements and historically nonworking closed-loop tracking. The architecture figure shows source components, not demonstrated operation. Component notices do not establish a single repository-wide license or wrapper/project-file terms.
