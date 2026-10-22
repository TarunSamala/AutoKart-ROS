# Senior reviewer report — autokart_system_monitor

A distinct, sequential review pass over the developer's code. Because this
package is new code (there was no existing algorithm to reuse), the review
looked specifically for ways the monitor could produce **false health
evidence**, in either direction: something healthy reported as stale, or
something dead reported as fresh. Both of the defects below are of that
kind, and both were reproduced against the pre-fix code before the fix was
trusted.

## Defect 1 (fixed): healthy best-effort publishers reported STALE forever

Heartbeat subscriptions were created with a plain depth `10`, which means
the default **RELIABLE** QoS. A publisher using **BEST_EFFORT** (the
sensor-data profile, used by e.g. `autokart_hp60c`'s
`/hp60c/{rgb,depth}/image_raw` and `camera_info`) is QoS-incompatible with a
reliable subscriber: DDS delivers nothing. The monitor would therefore have
reported a perfectly healthy dependency as `STALE` "never seen" for as long
as it ran, and the live config explicitly invites adding more topics.

- **Reproduced pre-fix:** a new test publishes `std_msgs/String` with
  `qos_profile_sensor_data` on a watched topic. Against the pre-fix code the
  dependency stayed STALE (assertion failed after 4 s), and ROS itself logged
  `offering incompatible QoS. No messages will be received from it. Last
  incompatible policy: RELIABILITY`.
- **Fix:** subscribe with best-effort, volatile QoS (`qos_profile_sensor_data`).
  That is compatible with both reliable and best-effort publishers, and
  dropping an occasional message is harmless for a liveness check.
- **Verified:** the new test,
  `test_node_integration.py::test_best_effort_publisher_is_detected_not_falsely_reported_stale`,
  passes, and the existing reliable-publisher tests still pass.

## Defect 2 (fixed): own output topics produced self-generated "fresh" evidence

If any of the monitor's own outputs (`/diagnostics`, `/system/health`,
`/system/startup_status`) is put in `heartbeat_topics`, the monitor receives
its own messages and reports that entry as permanently `fresh`. Adding
`/diagnostics` to a "watch everything" list is an easy configuration mistake,
and the result is false liveness evidence.

- **Reproduced pre-fix:** on a scratch copy with the guard disabled,
  watching `/diagnostics` with no other publisher reported `fresh`.
- **Fix:** own output topics are removed from `heartbeat_topics` at
  construction, with a logged warning, and recorded in
  `rejected_heartbeat_topics`.
- **Verified:**
  `test_fault_injection.py::test_own_output_topics_are_rejected_as_heartbeats`.

## Checked and found correct

- **Stopped dependency → STALE, not frozen OK.** Tested in-process
  (`test_a_dependency_that_starts_then_stops_publishing_is_tracked_correctly`)
  and again live: with `autokart_safety` (node 10) launched alongside this
  node, `/system/health` showed `/autokart/safety/status` as `fresh`; after
  the safety node process was killed it showed `stale: no message within
  timeout`. `/system/startup_status` stayed latched at `seen at least once`,
  as designed.
- **No dependency running → STALE "never seen", no crash, no hang**
  (`test_monitor_works_standalone_with_no_dependencies_running`; also seen in
  the live launch with nothing else running).
- **psutil missing → resources `WARN` with `"null"` values, never a
  fabricated number, never a crash.** Tested at the backend level and at the
  node level (the import is monkeypatched to fail).
- **First psutil CPU sample is meaningful:** `cpu_percent(interval=None)` is
  primed at construction. psutil documents that the first call returns a
  meaningless 0.0.
- **No authority:** the only publishers are the three `DiagnosticArray`
  topics. No command, request, or VCU topic exists anywhere in this package
  (grep-checked).
- **No hardcoded paths** anywhere in source or configs (grep-checked).

## Accepted, documented rather than fixed

- An empty `heartbeat_topics` makes startup vacuously "complete", and an
  empty YAML list may be typed by rclpy as a byte array. Both are documented
  in `CONTRACT.md` with the advice to configure at least one topic.
- `publish_rate_hz <= 0` raises at construction (a loud configuration
  error), as does an unknown `resource_backend` name. Failing loudly here is
  deliberate: a typo should be visible to the operator.
- A heartbeat means only that a message arrived. Payload validity is not
  inspected, deliberately: interpreting validity is the authority's job
  (node 10), not this reporter's.
