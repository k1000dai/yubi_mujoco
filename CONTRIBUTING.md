# Contributing

Keep installation small, behavior reproducible, and physical assumptions
explicit. Useful contributions include regression tests, clearer API boundaries,
measured calibration data with provenance, and improved collision models.

## Development setup

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) and use the
committed lockfile. The checkout defaults to Python 3.12; Python 3.10+ remains
supported. uv creates `.venv` automatically, without shell activation:

```bash
uv sync --locked --extra video
uv lock --check
uv run --locked --extra video ruff check .
uv run --locked --extra video ruff format --check .
uv run --locked --extra video pytest -q
```

Development tools live in `[dependency-groups].dev` and are included by default;
`video` remains a published optional extra. Use `uv add` for runtime dependencies,
`uv add --dev` for development tools, and `uv add --optional video` for video
requirements. Commit `pyproject.toml` and the generated `uv.lock` together. To
refresh a specific dependency intentionally, use `uv lock --upgrade-package NAME`,
then run the checks. Do not hand-edit the lockfile. CI rejects a stale lockfile.

Rendering tests need system OpenGL support. On headless Linux with EGL configured,
use `MUJOCO_GL=egl uv run --locked --extra video pytest -q`. See [usage](docs/usage.md) for backend and
viewer notes. Keep generated rollouts, build products, virtual environments,
checkpoints, credentials, and local caches out of commits.

## Changes and tests

Keep a change focused and explain the behavior it changes. Include a regression
test for bug fixes. Before opening a pull request:

1. Run lint and the relevant tests, then the complete test suite.
2. Run a base-install demo and reload a portable MJCF export if model/assets changed.
3. Check rendered overview and wrist images for visual changes.
4. Update the API/model documentation and changelog when behavior changes.
5. Report exact commands, platform, Python/MuJoCo versions, and anything untested.

For policy changes, preserve left/right ordering, hand-root frames, xyzw
quaternions, absolute motor radians, and the distinction between observations
and privileged `info`. Never make the scripted baseline appear to be a learned
policy. Preserve the 10 Hz versus 30 Hz distinction when changing timing.

For physics changes, do not directly move object free-joint state during actions
or introduce hidden grasp attachments. Maintain the open-hand negative control.
If a deliberate alternative model needs different behavior, expose and document
it rather than silently changing the meaning of existing results.

## CAD and third-party assets

Read [CAD provenance](docs/CAD_PROVENANCE.md) before regenerating geometry.
Runtime users should not need FreeCAD. Keep source STEP inputs and regeneration
tools in `cad/`, and keep only the 13 active material meshes in the installed
package. If topology changes intentionally, update the count, manifests, tests,
and documentation together.

Verify source hashes, unique solid membership, units, pivots, and local frames.
Do not replace geometry from memory or infer physical calibration from appearance.
Record exporter/tool versions and inspect articulation after regeneration.

Original simulation software is MIT; hardware-derived geometry and transform
data remain CERN-OHL-W-2.0. Preserve Toyota attribution, modification notices,
source access, and license texts. Upstream software references use Apache-2.0.
Do not label the complete distribution “MIT-only” or imply Toyota/AIRoA endorsement.

## Build and inspect distributions

```bash
uv build
uv run --locked --no-sync twine check --strict dist/*
uv run --locked --no-sync python scripts/check_dist.py dist
```

`uv build` builds the source archive first, then builds the wheel from that
archive. Isolated build dependencies are pinned separately in
`[tool.uv].build-constraint-dependencies`; `uv.lock` locks runtime and development
dependencies. Review and test updates to both when updating the build tools.

Check both wheel and source archive contents. The wheel must contain the active
meshes, JSON manifests, source/attribution notices, and relevant licenses. The
source archive must retain `uv.lock`, `.python-version`, and the CAD regeneration
inputs and tools. Neither should
contain local environments, credentials, experiment outputs, or caches.

Install the wheel into a clean environment outside the checkout. Test
`yubi-mujoco --version`, `demo`, `render`, and `export-mjcf` without the video
extra. Then install the video extra and test MP4 output. Follow the broader
[validation guide](docs/validation.md). A source-tree test pass does not prove
that the built wheel is complete.

## Releasing to PyPI

The release workflow is triggered by publishing a GitHub release. Preparing a
checkout, pushing a tag, or running a build alone does not establish that the
package has been published. Check the workflow result and the PyPI project
before announcing installation from PyPI.

Maintainer steps:

1. Choose the version and update `pyproject.toml`, the package version, and
   `CHANGELOG.md` together. Update the same-version sdist filename and PyPI URL
   in `src/yubi_mujoco/assets/SOURCE.md`; archive checks reject stale source links.
   Run `uv lock` to refresh the locked project version, then run the complete
   release checks on that exact commit.
2. In PyPI, create an upload token yourself. Store it in the GitHub repository's
   Actions secret named `PYPI_API_TOKEN`; never commit it or paste it into an
   issue, log, or chat. This workflow uses a token, not OIDC trusted publishing.
3. If the PyPI project does not exist yet, initial upload requires a token whose
   scope permits creating it. After the first successful publication, replace
   that credential with a project-scoped `yubi-mujoco` token and revoke the
   initial broader token. Follow [PyPI's API-token guidance](https://pypi.org/help/#apitoken).
4. Tag the validated commit with `v` plus the exact package version, for example
   `v0.1.0` for version `0.1.0`. Publish the corresponding GitHub release to
   trigger the release workflow.
5. Inspect the workflow's test, build, distribution-check, and upload results.
   Verify the expected version and files on PyPI, then install that version in a
   clean environment and run the public CLI smoke checks.

The configured workflow needs the repository secret; it does not require a
GitHub Environment. If maintainers want an additional manual approval boundary,
configure a protected `pypi` Environment and have the publishing job use it.
That is a separate repository/workflow configuration change, not provided by the
secret alone. Package publication and those settings remain maintainer actions.

PyPI normalizes underscores and hyphens in project names: `yubi_mujoco` is the
installation/import spelling used here, `yubi-mujoco` is the normalized project
name and console command. Do not upload a different artifact under an already
published version. Fix an error with a new version and document the change.
