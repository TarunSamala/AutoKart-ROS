#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-runs/dashboard-visual}"
scenarios=(
  straight_acceleration
  dashboard_research_lap
  dashboard_slalom
  vcu_wet_surface_stop
  vcu_steering_jam
  vcu_brake_cable_break
  vcu_motor_stuck_torque
  vcu_task_freeze
  vcu_physical_estop
  vcu_controlled_stop_nominal
  vcu_stop_timeout
  vcu_physical_estop_at_rest
)

for scenario in "${scenarios[@]}"; do
  PYTHONPATH=. python3 -m kart_sim.cli run "configs/scenarios/${scenario}.yaml" --output-dir "$output_dir"
done

echo "Dashboard replay set generated under: $output_dir"
