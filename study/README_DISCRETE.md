# Discrete cart–pendulum comparison

Run in MATLAB R2018b or newer with Simulink, from this directory:

```matlab
results = run_discrete_comparison;
```

This builds the models, runs paired baseline/pulse simulations, exports CSV/MAT
results, makes editable MATLAB figures and PNGs, and renders an MP4 animation.
No Control System Toolbox is required. Existing controller source files are
preserved; design MAT files and generated models are rebuilt from their equations.

## Models

* `cart_pendulum_discrete.slx`: six nonlinear discrete plants and six discrete
  controllers, all sampled at **0.0001 s**, using Simulink **FixedStepDiscrete**
  with fixed step **0.0001 s**. Each nonlinear plant has four discrete states and
  no continuous states. Its transition is classical RK4 with constant force
  over each step; RK4 is implemented inside the discrete state update, not as a
  continuous Simulink solver.
* `cart_pendulum_discrete_2ms.slx`: same discrete plant and fixed solver step,
  but original **0.002 s** controller period, to isolate plant discretization.
* `cart_pendulum_discrete_plants.slx`: standalone nonlinear RK4 plant and
  exact zero-order-hold linear discrete state-space plant driven by the same
  force. The linear model is valid near upright. The open-loop default initial
  angle is 5 degrees; the upright equilibrium is unstable.
* `cart_pendulum_staged_comparison.slx`: rebuilt previous continuous-plant,
  2 ms controller reference using the existing builder and controller unchanged.

## Fair comparison and interpretation

All six branches use the previous common PI startup for 0–5 s. At 5 s the
assigned controller activates and RLS coefficients freeze. A +1 N external
cart force acts from 8 to 8.1 s. Simulation duration is 20 s, initial angle is
5 degrees, and control force is limited to ±10 N. Each scenario also runs
without the pulse, allowing pulse-only responses to be calculated by subtraction.

Controllers: PI, LQR, P + RLS inverse DOB, P + ESO, P-only, and P + fixed biased
inverse DOB. This comparison uses the latest staged comparison in the workspace.
LQR retains the previous continuous-design gain, applied at discrete instants;
it is not redesigned as a discrete optimal controller. PI gains, filter time
constants and force limits are retained. ESO matrices are recomputed by exact
ZOH at the controller period. RLS forgetting is scaled as
`lambda = 0.9995^(Ts/0.002)` to preserve its decay per second; more measurements
per second still change adaptation. Each controller retains its causal
one-sample output delay; that delay is shorter at 0.1 ms.

Plant parameters: M=0.5 kg, m=0.2 kg, b=0.1 N s/m, l=0.3 m (pivot to pendulum
centre of mass), I=0.006 kg m², g=9.8 m/s². State order is `[x v theta omega]`.
No physical rail limit or sensor noise is modeled. Angle-only controllers do
not regulate cart position. ESO output is a lumped observer term, not an
independently verified measurement of external force.

## Outputs and reusable functions

`discrete_results/summary.csv` contains absolute and pulse-only measures.
`differences.csv` separates same-rate plant error from faster-controller changes.
Settling is measured from the end of the pulse and requires remaining inside
1 degree / 1 cm through the end, with at least one second of remaining record;
NaN means settling was not demonstrated. `first90deg_s` flags loss of upright.
Full-rate trajectories include controller internals and MATLAB x/y coordinates.
Reference continuous states are linearly interpolated; held controller outputs
use previous-value interpolation. Discrete trajectories are checked against
the exact simulation grid. Figures alone are decimated for display.

```matlab
s = load('discrete_results/results.mat');
figs = plot_cart_pendulum_results(s.results, 'discrete_results');
set(figs, 'Visible', 'on');
animate_cart_pendulum(s.results, '', 1); % interactive, real time
animate_cart_pendulum(s.results, 'my_animation.mp4', 4); % 4x playback
xy = cart_pendulum_xy(z, 0.3); % z is N-by-4
```

The animation shows all six fully discrete controller results using actual
world coordinates. Each panel follows its cart, with numerical x displayed so
drift remains evident. Rectangles represent carts; rods and markers represent
the pivot-to-centre-of-mass pendulum, not a claim about its full physical length.
The model's angle sign requires `bob_x = x - l*sin(theta)` and
`bob_y = l*cos(theta)`. Cart pivot y is zero. The trail shows the last 0.5 s.

Validation checks finite results, actuator limits, the exact 0.1 ms grid,
identical startup, RLS freeze, and the standalone model. The nonlinear discrete
step is compared with an independent mass-matrix ODE solution, and its numerical
Jacobian is compared with the exact-ZOH linear plant. See `validation.txt`.
