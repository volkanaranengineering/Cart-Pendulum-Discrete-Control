# Discrete cart-pendulum control study

A reproducible MATLAB/Simulink study of a nonlinear cart-pendulum plant and six controllers, using a **fixed-step discrete solver at 0.0001 s**. This companion repository preserves the complete results from **3 October 2026**, source code, saved models, plotting functions, and a MATLAB x/y animation.

Research direction: **Volkan Aran**. Code, analysis, repository preparation, and tutorial developed with **OpenAI Codex under human direction**.

## Start here

- **[Illustrated Turkish tutorial (PDF)](output/pdf/Ayrik_Araba_Sarkac_Ders_Notu_TR.pdf)** and its [editable Markdown source](docs/TUTORIAL_TR.md).
- [Turkish repository guide](docs/README_TR.md).
- [Measured comparison](study/discrete_results/COMPARISON.md), [full score table](study/discrete_results/summary.csv), and [validation record](study/discrete_results/validation.txt).
- [Six-controller 2D animation](study/discrete_results/controllers_2d.mp4), at 4x playback speed.
- [Reproduction instructions](docs/REPRODUCIBILITY.md), [study guide](docs/STUDIES.md), and [data catalog](docs/DATA_CATALOG.csv).

![Controller angle comparison](study/discrete_results/angle_comparison.png)

## What was compared?

| Variant | Plant | Solver step | Controller period |
|---|---|---|---|
| Previous | Continuous nonlinear | Variable-step ode45, maximum 0.001 s | 0.002 s |
| Discrete / original rate | Nonlinear discrete RK4 | 0.0001 s | 0.002 s |
| Fully discrete | Nonlinear discrete RK4 | 0.0001 s | 0.0001 s |

Each variant contains PI, LQR, P + RLS inverse DOB, P + ESO, P-only, and P + fixed inverse DOB. All branches share PI startup until 5 s; assigned controllers activate and RLS freezes at 5 s. Each 20 s scenario runs both without a pulse and with a +1 N pulse at 8–8.1 s. This gives **36 controller trajectories** (three variants, two scenarios, six controllers).

The LQR gain is the previous continuous-design gain sampled at the new period, not a newly optimized discrete LQR. Filters and the ESO discretization follow the controller sample period; RLS forgetting is scaled per elapsed time. The one-sample control delay decreases with the period. See the tutorial before attributing every change solely to plant discretization.

## Selected results

Absolute peak angle over 8–20 s, including residual startup/switching response:

| Controller | Previous peak (deg) | Fully discrete peak (deg) | Fully discrete final cart x (m) |
|---|---:|---:|---:|
| PI | 16.715 | 5.260 | -1.022 |
| LQR | 0.705 | 0.663 | approximately 0 |
| RLS DOB | 8.530 | 3.643 | -0.168 |
| ESO | 1.352 | 1.225 | -0.949 |
| P-only | 9.952 | 5.001 | -1.623 |
| Fixed inverse DOB | 1.080 | 0.954 | 0.531 |

LQR regulates both angle and cart position in this scenario. Angle suppression alone does not imply cart-position regulation. Pulse-only measures use paired baseline subtraction and are separately reported. No branch crossed 90 degrees during the recorded runs. These results do not establish general robustness or hardware performance.

## Layout

```text
study/                   Runnable MATLAB sources and saved Simulink models
  KontrolAI/             Original continuous plant builder and model
  discrete_results/      Complete 2026-10-03 trajectories, MAT archive, figures, video
docs/                    Turkish tutorial, study guide, provenance, reproduction
output/pdf/              Illustrated Turkish tutorial
tools/                   Archive verification, manifest builder, PDF renderer
manifest.json            File sizes and SHA-256 hashes (excluding itself and Git)
requirements-docs.txt    Optional PDF-generation dependencies
```

The relative MATLAB layout follows the companion TMATS research archives. This project itself has **no T-MATS dependency**. Original workspace files are copied into this independent repository; the existing workspace is not moved.

## Clone, verify, and run

Full-resolution CSVs and the large MAT archive use **Git LFS**. Obtain the actual data, not just LFS pointer files:

```sh
git lfs install
git clone https://github.com/volkanaranengineering/Cart-Pendulum-Discrete-Control.git
cd Cart-Pendulum-Discrete-Control
git lfs pull
python tools/verify_archive.py
```

The Python verifier uses only the standard library. It checks archive hashes and independently recomputes the six fully discrete controllers' pulse-response metrics from saved CSVs. This is an archive/numerical check, not a fresh MATLAB simulation.

To run from MATLAB R2018b with Simulink, preferably in a disposable clone so the archived results stay intact:

```matlab
cd study
addpath(pwd, fullfile(pwd,'KontrolAI'))
results = run_discrete_comparison;
```

No Control System Toolbox is required. Re-running overwrites generated models, designs, and `study/discrete_results/` outputs. For viewing saved figures and animation without simulating, see [reproduction](docs/REPRODUCIBILITY.md).

## Provenance and reuse

See [AI contribution and limitations](docs/AI_DISCLOSURE.md) and [notices](THIRD_PARTY_NOTICES.md). MATLAB and Simulink are external proprietary dependencies and are not redistributed. No blanket license is assigned without the author's choice. This is a simulation research archive, not an independently peer-reviewed or hardware-qualified control package.
