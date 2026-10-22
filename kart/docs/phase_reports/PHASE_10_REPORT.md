PHASE:
Phase 10 — Fault-Injection Benchmark

SOFTWARE FRAMEWORK:
COMPLETE

TESTED CATALOG COVERAGE:
PARTIAL

OVERALL STATUS:
PARTIAL

VERDICT:
APPROVED_WITH_CONDITIONS

IMPLEMENTED LOCALLY:
- Traceable catalog schema, deterministic runner, telemetry-based safety
  oracle, timing/safe-state metrics, coverage report and resumable result files.
- Stable pairwise combinations, seeded parameter sampling and deterministic
  one-minimal failure reduction.
- Critical physical smoke cases for brake cable break, stuck torque and VCU
  freeze.

TESTED LOCALLY:
- Critical smoke: 3/3 pass, 3/3 catalog IDs covered, zero invariant violations.
- Resume, pairwise, seeded sampling, minimization and oracle violation tests.

NOT EXECUTED:
- Exhaustive combinatorial coverage or statistically representative
  real-hardware failure distributions.

REASON:
- Exhaustive safety coverage is not a defensible finite claim; hardware
  distributions are unavailable.

COMMAND TO VERIFY:
- `PYTHONPATH=. python3 -m kart_sim.cli safety campaign --output-dir runs/safety-campaign`

KNOWN LIMITATION:
- Tested fault coverage is explicitly not “100% safety coverage.”
