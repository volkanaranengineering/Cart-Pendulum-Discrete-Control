# Contribution and limitations

Volkan Aran requested the discrete models, comparison with previous controllers, plotting, animation, a separate repository, and a Turkish tutorial. OpenAI Codex generated and revised the implementation, ran local MATLAB simulations and checks, packaged the archive, and drafted the tutorial under that direction.

The repository records simulations and numerical checks, not independent human peer review or hardware experiments. The fixed step describes numerical time; no embedded real-time execution deadline was benchmarked. State feedback is idealized. Measurement noise, quantization, hard rail limits and unmodeled actuator dynamics were not evaluated. The 20 s record does not prove asymptotic stability or robustness.

Provenance: current workspace discrete study and its required staged-controller/continuous-plant dependencies, archived on 2026-10-03. The TMATS repositories supplied an organizational reference only; no NASA T-MATS model or library is required or copied here.
