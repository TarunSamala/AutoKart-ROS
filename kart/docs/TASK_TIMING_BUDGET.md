# Task Timing Budget

These are simulation acceptance budgets, not real-kart deadlines.

| Task | Rate | Average budget | Deadline | Severity |
| --- | ---: | ---: | ---: | --- |
| Physics aggregate | 1000 Hz | 0.700 ms | 1.000 ms | warning |
| VCU | 1000 Hz | 0.400 ms | 1.000 ms | error |
| Independent safety | 1000 Hz | 0.100 ms | 1.000 ms | critical |
| Gazebo follower apply | 100 Hz | 4.000 ms | 10.000 ms | warning |
| RGB / depth | preset | preset latency | sensor period | warning |
| LiDAR | preset | preset latency | sensor period | warning |
| IMU | 200 Hz | 0.300 ms | 5.000 ms | warning |

First workstation full-lap timing (`realtime_qualified`):

| Task | Mean | p95 | p99 | Maximum | Misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| Physics aggregate | 0.559 ms | 0.822 ms | 1.034 ms | 3.895 ms | 222 warnings |
| VCU | 0.149 ms | 0.229 ms | 0.299 ms | 0.759 ms | 0 |
| Independent safety | 0.0025 ms | 0.0036 ms | 0.0046 ms | 0.189 ms | 0 |

The physics aggregate includes VCU, independent safety, actuator dynamics, and
plant work, so its warning does not mean those inner tasks were skipped.
No physics step was dropped. A physics warning is not silently relabelled as a
sensor drop.

Exact post-regression repeat (`realtime_final`):

| Task | Mean | p95 | p99 | Maximum | Misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| Physics aggregate | 0.592 ms | 1.036 ms | 1.370 ms | 2.325 ms | 1,037 warnings |
| VCU | 0.159 ms | 0.289 ms | 0.447 ms | 1.256 ms | 2 |
| Independent safety | 0.0024 ms | 0.0035 ms | 0.0048 ms | 0.265 ms | 0 |

Because the VCU zero-miss result did not repeat, the workstation status is
`REALTIME_DEGRADED`. The 1 ms deadline remains unchanged.
