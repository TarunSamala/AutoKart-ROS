PHASE:
Phase 14 — Real-Kart Calibration and Validation

SOFTWARE FRAMEWORK:
COMPLETE

REAL-DATA VALIDATION:
NOT_EXECUTED_EXTERNAL_DEPENDENCY

OVERALL STATUS:
PARTIAL

VERDICT:
MORE_DATA_REQUIRED

IMPLEMENTED LOCALLY:
- Dataset metadata/schema, immutable checksum, frozen calibration/validation/
  holdout split registry, typed parameter registry, CSV import, bounded linear
  recovery, deterministic bounded optimizer and report template.

TESTED LOCALLY:
- Split leakage rejection, parameter bounds, identifiability failure,
  deterministic synthetic recovery and claim-state template.

NOT EXECUTED:
- Real logs, time alignment, parameter fitting, uncertainty from physical data,
  independent held-out validation and sim-to-real evaluation.

REASON:
- Real-kart data and measurements are unavailable.

COMMAND TO VERIFY WHEN AVAILABLE:
- Run the future native dataset test with an immutable supplied dataset and
  follow `docs/REAL_TEST_PLAN.md`.

DIGITAL TWIN CLAIM:
NOT JUSTIFIED
