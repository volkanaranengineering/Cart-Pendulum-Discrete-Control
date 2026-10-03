# Reproduction and environment

Snapshot date: 2026-10-03, Europe/Istanbul. Original simulations ran in MATLAB R2018b on Windows with Simulink. No Control System Toolbox functions are called: the continuous LQR gain is recovered from the Hamiltonian stable eigenspace; ZOH matrices use `expm`.

Packaging validation loaded all four saved comparison/standalone models from this separate repository and ran each for 0.02 s; see `RELOCATION_VALIDATION.txt`. The batch wrapper was corrected to use the R2018b-compatible `bdclose('all'); exit(code)` shutdown sequence. This changes process shutdown only, not the archived controller or plant equations.

## Preserve the snapshot

Use a fresh working clone for simulation. `run_discrete_comparison` changes MATLAB's working directory to `study`, rebuilds saved models/design files, and overwrites the generated contents of `study/discrete_results`. The main repository is the archived record. No absolute original-workspace paths are required by the packaged MATLAB sources.

Clone with Git LFS and run `python tools/verify_archive.py` from the repository root. `--hash-only` omits CSV metric recomputation. The verifier requires Python 3.10+ and no third-party packages. A missing/LFS-pointer file causes verification to fail explicitly. GitHub's source ZIP may not contain hydrated LFS data; prefer `git clone` plus `git lfs pull`.

## Full simulation

Start a fresh MATLAB session to avoid an older cart-pendulum model with the same name on the path or already loaded. From the repository root:

```matlab
cd study
addpath(pwd,fullfile(pwd,'KontrolAI'))
which build_discrete_comparison -all
results = run_discrete_comparison;
```

Allow several minutes and several GB of disk and memory for full-resolution output. The runner builds the continuous comparator, two discrete comparison models, and standalone nonlinear/linear plants. It performs six Simulink runs (three variants times two disturbances), each with six branches. The final call renders a 4x-speed MP4 through `VideoWriter`; interactive MATLAB graphics and an MPEG-4 encoder are needed for video generation. The completed reference run and graphics were produced on Windows; other MATLAB/platform combinations have not been validated.

For only rebuilding models: `build_discrete_comparison`. For an independent plant-equation check after rebuilding: `validate_discrete_models('discrete_results')`. The latter checks the plant equations, local linearization and standalone run; full-run assertions are in `run_discrete_comparison`.

## View results without rerunning

```matlab
cd study
addpath(pwd,fullfile(pwd,'KontrolAI'))
s = load('discrete_results/results.mat');
figs = plot_cart_pendulum_results(s.results, 'discrete_results');
set(figs,'Visible','on');
animate_cart_pendulum(s.results,'',1);
% Export a separate video:
animate_cart_pendulum(s.results,'controllers_replay.mp4',4);
% Open saved model:
open_system('cart_pendulum_discrete.slx');
```

`plot_cart_pendulum_results` and `animate_cart_pendulum` also accept the MAT-file path directly. Saved `.fig` files can be opened using `openfig` without loading the large results archive.

## Rebuild the Turkish PDF

Install the optional dependencies in `requirements-docs.txt`, then run `python tools/build_tutorial_pdf.py`. The builder uses the Turkish Markdown source and existing result figures. It searches for Arial on Windows or DejaVu Sans on Linux; see the script if other fonts are required. The PDF output is under `output/pdf`. Rendering/visual QA used Poppler; Poppler is external and not bundled.

## Update an intentional new study record

Do not overwrite the 2026-10-03 record to represent a different experiment. Create a new result directory or branch, document its parameters, then run `python tools/build_manifest.py` after intentional archive changes and `python tools/verify_archive.py`. The manifest covers all nonignored repository files except itself. Git LFS must be enabled before committing full-resolution result CSVs and `results.mat`.
