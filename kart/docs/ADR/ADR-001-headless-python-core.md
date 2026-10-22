# ADR-001: Deterministic Python Headless Core

**Status:** accepted for initial implementation only.

**Purpose:** make Phase 1–2 executable locally.

**Verdict:** APPROVED_WITH_CONDITIONS.

Conditions: physics and safety interfaces must remain separable from Python-specific runtime concerns; performance and timing must be measured before SIL/HIL claims; Project Chrono/C++ remains an optional future backend, not a dependency for the baseline.
