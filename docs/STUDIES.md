# Canonical study and data dictionary

The canonical snapshot is `study/discrete_results`, dated **2026-10-03**. The preserved model/controller lineage comes from the prior staged six-controller comparison. Earlier independent PI/LQR/ADOB studies are not being republished wholesale.

## Models and source map

| File under study/ | Role |
|---|---|
| `build_discrete_comparison.m` | Rebuild all comparison/standalone models and design matrices |
| `discrete_cart_plant.m` | Four-state discrete nonlinear RK4 update |
| `discrete_staged_controller.m` | Sample-period-aware version of previous six-mode controller |
| `staged_controller.m` | Unchanged previous controller, 0.002 s period |
| `build_staged_comparison.m` | Unchanged continuous-plant reference builder |
| `KontrolAI/build_cart_pendulum.m` | Original continuous nonlinear plant block builder |
| `run_discrete_comparison.m` | Paired scenarios, metrics, exports and run assertions |
| `validate_discrete_models.m` | Independent mass-matrix ODE and linearization checks |
| `plot_cart_pendulum_results.m` | Reusable comparison plots |
| `cart_pendulum_xy.m` | World-coordinate cart pivot and pendulum centre of mass |
| `animate_cart_pendulum.m` | Six-panel x/y animation and MP4 export |

All `.slx` and small design `.mat` files needed to inspect saved models are included. Runtime caches (`slprj`, `.slxc`, autosaves), installed dependencies, and machine-specific MATLAB startup logs are excluded.

## Result files

`{variant}_{controller}_{scenario}.csv`: 36 full-resolution files, each with 200001 samples from 0 to 20 s inclusive at 0.0001 s spacing. Variants are `previous`, `discrete_2ms`, `discrete_01ms`; scenarios are `baseline` and `kick`.

| Columns | Meaning |
|---|---|
| `t` | Time, s |
| `x,v,theta,omega` | Cart position (m), velocity (m/s), upright angle (rad), angular velocity (rad/s) |
| `u,raw,dhat` | Limited control, unlimited command, compensation term |
| `a_hat,h_hat,b_hat` | Inverse-model estimates for M+m, m*l, friction |
| `innovation,updated` | Filtered regression residual and adaptation-update flag |
| `phi1,phi2,phi3,uf` | Filtered regression components and filtered applied actuator force |
| `omega_hat,f_hat` | ESO observer states; interpretation follows the observer design |
| `disturbance` | External cart force, N, added after actuator saturation |
| `cart_x,cart_y,bob_x,bob_y` | World x/y coordinates, m; bob denotes pendulum centre of mass |

`dhat` is not a uniformly physical force estimate across controller modes. The ESO's third state is a lumped acceleration term under design normalization b0=1; its numerical subtraction follows the inherited controller implementation.

`results.mat` contains `results.t`, `results.runs{variant,scenario,controller}`, labels, summary, differences, Ts and l. Each run is N-by-19: the CSV columns from x through disturbance. The four geometry columns are generated separately. Variant indices are 1=previous, 2=discrete_2ms, 3=discrete_01ms; scenario indices are 1=baseline and 2=kick. Controller ordering is PI, LQR, RLS_DOB, ESO, P_only, Fixed_inverse.

`summary.csv` has 18 rows. Absolute peaks and force RMS use 8–20 s. Pulse-only metrics subtract the corresponding baseline trajectory. Settling uses 1 degree / 1 cm bands from 8.1 s and requires the signal to remain within the band until 20 s with at least one second remaining. Zero means it never left the band after 8.1 s; NaN means settling was not demonstrated. `first90deg_s` is the first absolute angle at or above 90 degrees, or NaN if absent.

`differences.csv` gives maximum full-trajectory differences from the previous run; same-rate columns isolate plant discretization plus reference-output interpolation, while fast-rate columns include controller-period changes.

`validation.txt` records the MATLAB run checks. PNG/FIG pairs show angle, cart position, force, paired pulse response, and pendulum COM trajectory. `controllers_2d.mp4` shows 20 simulated seconds in approximately 5 video seconds (4x speed). `animation_preview.png` is its initial arrangement.
