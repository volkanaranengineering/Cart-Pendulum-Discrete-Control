# Measured controller comparison

20 s simulations; common PI startup to 5 s; +1 N pulse at 8–8.1 s.
All new plants use a discrete RK4 transition and fixed-step discrete solver
at 0.0001 s. The new controller sample period is also 0.0001 s.

| Controller | Previous peak angle after 8 s (deg) | New peak angle after 8 s (deg) | New final cart x (m) |
|---|---:|---:|---:|
| PI | 16.715 | 5.260 | -1.022 |
| LQR | 0.705 | 0.663 | approximately 0 |
| RLS DOB | 8.530 | 3.643 | -0.168 |
| ESO | 1.352 | 1.225 | -0.949 |
| P only | 9.952 | 5.001 | -1.623 |
| Fixed inverse DOB | 1.080 | 0.954 | 0.531 |

These are absolute post-pulse peaks, including any ongoing startup/switching
response. Paired simulations without the pulse isolate its incremental effect;
see `summary.csv` and `discrete_response.png` for those measures.

LQR regulates both cart position and pendulum angle in this scenario.
Smaller angular excursions for the angle-only controllers do not imply cart
position regulation. None of the simulated branches crossed 90 degrees.

Keeping the original 2 ms controllers on the new discrete plant gives a
maximum comparison-grid difference across the six controllers of 0.000140 m
and 0.01284 degrees. Those differences include interpolation of the previous
continuous solver output. The materially larger changes with 0.1 ms control
include the effect of reduced one-sample delay and changed adaptation sampling;
the gains were not retuned. This is a scenario comparison, not a general
stability or optimality guarantee.

Validation passed: exact discrete time grid, finite outputs, force limits,
common startup, adaptation freeze, standalone models, and x/y sign convention.
The RK4 plant matched an independent mass-matrix ODE over a 0.1 s test within
2.23e-15 in state magnitude. Its local Jacobian matched the exact-ZOH linear
model within 1.12e-16. See `validation.txt` for the recorded checks.
