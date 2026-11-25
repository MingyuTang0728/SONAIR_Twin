# Evaluation Card - SONAIR v2

**What is computed.** Per test run and per track, the per-sample error of the entry and of
S0 against the real target - |Δ current| in A; ‖Δ angular rate‖ in rad/s. GCR = 1 − p95(entry)
/ p95(S0) (and the same at the median), averaged over runs. Tail scores use only the tail
samples. Robustness is the same score on each stressed input. Calibration, when `sigma` is
given, is the share of samples whose error lies inside ±1.645σ (0.90 is calibrated).

**Headline.** Effort Tail GCR-p95: the share of S0's worst-moment error in the elbow current that an entry removes, on the long-tail samples of the held-out cells.

**Fairness.** S0, the reference, is computed by the evaluator from the public inputs with the
same code as every entry. Baselines are made from the public package only and submitted as
entries. Anything fitted (the effort map, baseline coefficients) uses the train split only.

**Integrity.** The held-out targets never leave the evaluator. Submissions use a team API key
(a key that grants nothing else), at most 5 scored a day; the same validator runs at home and on
the server; the evaluator ships as a container so a score can be reproduced bit for bit.

**Reporting.** Every scorecard lists every held-out cell, the worst included.

**Versioning.** Changing the task, the splits or the metric is a new version; scores across
versions are not compared.

_Generated 2026-10-12 18:44 UTC._
