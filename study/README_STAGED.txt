STAGED COMPARISON
Run in MATLAB from this folder:
  build_staged_comparison
  run_staged_comparison
Open cart_pendulum_staged_comparison.slx.

0-5 s: all branches use identical PI (Kp20 Ki1, 2ms sample).
RLS trains ONLY from disturbance-free applied control force.
At5 s: coefficients freeze; assigned control and DOB branches activate.
At8-8.1 s: +1N external cart pulse. End20 s.
Matched no-kick simulations isolate the pulse from the switching transient.
Six branches: PI, LQR, P+RLS inverse DOB, P+fixed ESO,
P-only, P+fixed biased inverse DOB.
Outputs: staged_results trajectory CSVs, summary, validation, MAT archive.
Report: Staged_PI_RLS_DOB_Comparison.pdf (13 pages).

The five-second transient did not accurately identify friction. Learned
coefficients [0.6880064 0.0530024 0.0294777], true [0.7 0.06 0.1].
Low training residual is not evidence of correct parameters.
Angle-only controllers do not regulate cart position. See paired and
absolute results separately. No noise or physical rail limits modeled.
