# Takeover integration contract (2026-10-22)

This existing diagnostics implementation is extended, with no additional
monitor process. All canonical critical status topics are configured. Dynamic
subscriptions use best-effort QoS to receive sensor and reliable publishers.
`/diagnostics`, `/system/health`, `/system/startup_status` remain
DiagnosticArray. No command/request publisher exists.

Each observed topic retains a bounded 128-arrival window and reports actual
arrival rate, source stamp age, nonmonotonic stamp count, and the latest
upstream payload (including model/calibration hashes, latency/drop counters,
state, transition/fault reasons and sensor quality when provided). Fresh
invalid payloads produce an ERROR evidence component. CPU/RAM reuse psutil;
missing backend reports UNAVAILABLE. USB reset and queue backlog are null
unless the upstream source reports corresponding counters; no values are
invented. ROS timestamp age remains clock_domain_verified=false. Arrival
freshness is not source timestamp freshness or physical qualification.

Startup status is an explicitly historical seen-once latch. It does not
regress after dropout and must never be used as current health or an arming
gate. `/system/health` supplies current per-topic age and evidence instead.
Bag recording remains a bringup responsibility; this package creates no
extra camera/USB/serial owner or recorder.

## Historical package contract

