# Senior reviewer report — autokart_safety

Performed as a distinct, sequential review pass over the developer's actual
source, by actually running the full test suite end to end (not just reading
the diff) and investigating every failure down to a root cause, per the
mission statement's instruction that this must be genuine, not a rubber
stamp.

## Defect found and fixed

**`adapter.load_safety_core()` could permanently shadow the real dependency
with a stale, previously-tried path.**

While running this package's own test suite for the first time end to end
(`pytest test/` across all three test files in one process), every test that
loaded the real `autokart_vcu_safety` core **after** an earlier test had
already attempted a different (in that case, deliberately invalid/stub)
`vcu_safety_package_path` started failing with
`SDK_ADAPTER_API_MISMATCH:missing=SafetyLogic,Inputs,Config,VcuView,evaluate_startup`
— i.e. the adapter was returning a stub module instead of the real core, even
though it had been given the correct path.

Root cause, confirmed by direct reproduction (see below): the original
implementation only inserted a new dependency path into `sys.path` **if it
was not already present** (`if src_str not in sys.path: sys.path.insert(0,
src_str)`), and only cleared the `sys.modules` cache, not `sys.path`
ordering. Once some earlier call had already inserted a *different* path at
the front of `sys.path`, a later call whose own path happened to already be
present (just further back) would leave that older, wrong path in front.
Since Python resolves a regular (non-namespace) package from the *first*
matching `sys.path` entry and does not consider the rest, every subsequent
import of `autokart_vcu_safety.core` kept resolving to whatever package sat
at the front — in the reproduction below, a test stub — never the real core,
**silently**, with no exception and no obviously-wrong-looking log line.

For a node whose entire purpose is "never silently misjudge whether it is
safe to arm," an adapter that can silently and permanently load the wrong
implementation of the arming logic is a serious defect, not a cosmetic one —
even though in this specific instance it happened to manifest as a *visible*
`SDK_ADAPTER_API_MISMATCH` failure (the stub it fell back to lacked the
expected attributes) rather than a silently-accepted wrong-but-valid-looking
core. A differently-shaped stale path (one that happened to also expose
`SafetyLogic` etc., e.g. an older checked-out copy of the real package left
on a developer's machine from a previous session) would not have raised at
all — it would have quietly run the *wrong* arming logic.

**Reproduced before trusting the fix:** the failure was first observed
directly (the full suite failing as described above, before any fix). An
intermediate fix that only purged `sys.modules` still left the full suite
failing — that is what exposed the `sys.path`-ordering root cause rather
than the cache alone. The precise triggering sequence is: real path loaded
(now on `sys.path`) → a different path loaded (inserted in front of it) →
real path requested again (pre-fix code skips re-insertion because it is
"already present", so the other path keeps winning). See VALIDATOR_REPORT.md
item 4 for the independent revert-and-rerun, including a correction to the
regression test itself (its first version was order-dependent).

**Fixed**: `load_safety_core` now always moves the resolved dependency path
to the *front* of `sys.path` (removing any existing occurrence first, not
just skipping insertion when already present) in addition to purging any
cached `autokart_vcu_safety*` entries from `sys.modules`. This guarantees the
path given to *this* call always wins, regardless of what any earlier call
did. **Verified** with a new regression test,
`test/test_adapter.py::test_reimport_after_a_bad_path_loads_the_new_real_module`,
which reproduces the exact stub-then-real sequence and asserts the real
`SafetyLogic` class comes back, not a stale stub. Also verified this was the
root cause of every other test in `test_fault_injection.py` and
`test_node_integration.py` incorrectly reporting `SDK_ADAPTER_API_MISMATCH`
for a correctly-specified real path — all of them passed once this one fix
landed, with no other change.

## Checked and found correct

- **Default-DISARMED / never-auto-arm**: confirmed by direct test
  (`test_node_integration.py::test_safety_node_never_auto_arms_on_startup_with_real_core`)
  that feeding every upstream input as fresh/valid, with the real core
  loaded, but sending **no** operator request at all, leaves the node at
  `DISARMED` across multiple ticks. Also confirmed the dependency-unavailable
  path (`state=UNAVAILABLE`) can likewise never reach `ARMED` — there is no
  code path in `_tick()` that calls `self.logic.step(...)` when `self.logic
  is None`.
- **Ready-but-unverified arm is refused**: directly exercised
  (`test_fault_injection.py::test_ready_but_unverified_arm_attempt_is_rejected`)
  with every freshness gate satisfied and only the three `*_verified` flags
  left at their false default — arm refused, `"ARM"` never forwarded.
- **Fail-closed repeats, not one-shot**: confirmed
  `_publish_unavailable()` is called from every `_tick()` while
  `self.logic is None`, at the same 0.1s period as normal operation — not a
  single warning followed by silence, matching the lesson from
  `autokart_hp60c`'s own reviewed fix for the same failure class.
- **No path in source is hardcoded to this repository's location**: grepped
  the package for `/home/mark1` and absolute paths; the only occurrences are
  in `config/safety_live.yaml`'s comments (documentation, not an active
  default — the actual parameter value is `""`) and in this report/
  DEPENDENCIES.md prose.
- **`vehicle_authorized` stays `false` in every published payload**,
  including the positive "arm succeeds" test path — confirmed by directly
  asserting on the published JSON, not just by reading the source.
- **The dashboard gateway's only path to this node is the bounded
  `/autokart/safety/request` whitelist**: confirmed `_on_request` drops
  anything outside `ALLOWED_REQUESTS` with a logged warning, matching the
  identical whitelist already enforced independently by
  `autokart_dashboard_gateway`'s `ALLOWED_SAFETY_REQUESTS` (node 11, reviewed
  separately in that package's own reports) and by the original
  `autokart_vcu_safety.node`.

## Not independently re-verified by this reviewer

- The reused `SafetyLogic`/`evaluate_startup` state-machine logic itself
  (stop-latch timing, arming timeout, demo-drive gating, etc.) was authored
  and already reviewed before this session and is unchanged by this package
  — this review did not re-derive or re-test that logic beyond what the
  adapter's own loading of it requires, consistent with "do not
  re-litigate working algorithms unnecessarily." Its own test suite
  (`kart/ros2_ws/src/autokart_vcu_safety/test/test_safety_core.py`) was read
  and confirmed to already cover it; this review ran it (see
  VALIDATOR_REPORT.md) rather than assuming it still passes.
- Real VCU firmware/ESP32 behavior on receiving `ARM`/`DIS`/`RST` — no
  hardware in this environment; out of scope for this node regardless.
