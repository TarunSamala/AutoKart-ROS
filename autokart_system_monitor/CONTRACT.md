# autokart_system_monitor — interface contract

## What this node is

A health and startup-evidence **reporting** node. It tracks ROS node
heartbeat/liveness and basic CPU/RAM, and publishes them as standard
`diagnostic_msgs/DiagnosticArray`.

## What this node is NOT

**This node has no safety authority.** It cannot arm, disarm, stop, gate,
or authorize anything, and it has no publisher to any command, request, or
VCU topic. A `WARN`/`ERROR`/`STALE` reading here is information for a
person or a dashboard — never an automatic vehicle action. Contrast with
`autokart_safety` (node 10), which IS the software arming authority (and
which is itself not the physical E-stop; see that node's `CONTRACT.md`). If
anything in the system must *act* on staleness, that decision belongs to
node 10 (which already applies its own freshness gates to its own inputs),
not to this node.

## Topics published

All three use the standard `diagnostic_msgs/DiagnosticArray` (no new message
type was introduced; `autokart_msgs/SensorDeliveryMetadata` was considered
and rejected because it is a per-sensor delivery record, not a generic
multi-component health report).

| Topic | Rate | Meaning |
|---|---|---|
| `/diagnostics` | `publish_rate_hz` (default 1 Hz) | live snapshot, standard ROS diagnostics convention (consumable by `rqt_robot_monitor`/`diagnostic_aggregator`) |
| `/system/health` | same | the same live snapshot in this node's own namespace |
| `/system/startup_status` | same | one-way latch per tracked topic: `OK` once that topic has been seen **at least once** since this node started, never regressing afterwards. Answers "did the system finish coming up", not "is it healthy now" |

Each `DiagnosticStatus`: `name = "autokart_system_monitor: <topic or 'resources'>"`,
`hardware_id = "autokart_system_monitor"`, `level` one of
`OK(0)`/`WARN(1)`/`STALE(3)` (`ERROR(2)` is reserved, not currently
emitted), `values` carry `age_s`/`timeout_s` (heartbeats) or
`cpu_percent`/`ram_percent` (resources; `"null"` when unavailable, never a
fabricated number).

## Heartbeat tracking (subscriptions)

For each topic in `heartbeat_topics` (default `/hp60c/status`,
`/autokart/safety/status`), the node resolves the topic's message type
**dynamically from the live ROS graph** (`rosidl_runtime_py`) and subscribes
once it appears; any message on it counts as a heartbeat. Retried every
`topic_discovery_period_s`, so:

- a dependency that is not running (or never runs) is reported `STALE`
  "never seen" — never an error, never a crash; this node works standalone;
- a dependency that was fresh and then stops is reported `STALE` "stale: no
  message within timeout" after `heartbeat_timeout_s`;
- adding a new topic to watch needs only a config change, no new code.

Subscriptions use **best-effort, volatile** QoS (sensor-data profile) so
they are compatible with both reliable and best-effort publishers. Using
the default reliable QoS was a real defect found in review (a healthy
best-effort publisher was reported STALE forever) — see
`reports/SENIOR_REVIEWER_REPORT.md`.

## Resources

`resource_backend: "psutil"` (default) uses `psutil` if importable, else
falls back to reporting `WARN` "resource sampling unavailable:
PSUTIL_IMPORT_FAILED..." — the node never crashes and never fabricates a
reading for lack of psutil. `"none"` disables sampling explicitly. Any
other value is a configuration error and **fails loudly at construction**
(a typo should be visible, not silently treated as "unavailable").

## Parameters

See `config/system_monitor_synthetic.yaml` / `config/system_monitor_live.yaml`:
`heartbeat_topics`, `heartbeat_timeout_s` (one value for all topics),
`resource_backend`, `cpu_warn_pct`, `ram_warn_pct`, `publish_rate_hz`,
`topic_discovery_period_s`.

## Known limitations

- One shared `heartbeat_timeout_s` for every topic (no per-topic timeout).
- "Heartbeat" = any message arrived; it does not inspect payload validity
  (e.g. an `/hp60c/status` saying `quality=INVALID` still counts as alive).
  Payload validity is the consuming authority's job (node 10).
- `heartbeat_topics: []` makes `/system/startup_status` report fully started
  immediately (vacuously), and an empty YAML list may be typed by rclpy as a
  byte array rather than a string array; configure at least one topic.
- `publish_rate_hz <= 0` is a configuration error (fails at construction).
