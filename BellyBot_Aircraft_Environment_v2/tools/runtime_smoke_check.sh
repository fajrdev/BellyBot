#!/usr/bin/env bash
# On a Gazebo Harmonic host: syntax checks and one simulated second per world.
# Process success is not a contact, direction, throughput or safety test.
set -euo pipefail
asset_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v gz >/dev/null || { echo 'Gazebo CLI unavailable' >&2; exit 2; }
export GZ_SIM_RESOURCE_PATH="$asset_root/ros2/bellybot_aircraft_environment/models:$asset_root/ros2${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
log_dir="${1:-$asset_root/runtime_logs}"
mkdir -p "$log_dir"
for world_file in "$asset_root"/ros2/bellybot_aircraft_environment/worlds/*.sdf; do
  world_name="$(basename "$world_file" .sdf)"
  gz sdf -k "$world_file" >"$log_dir/$world_name.schema.log" 2>&1
  timeout 60 gz sim -s -r --iterations 1000 "$world_file" >"$log_dir/$world_name.sim.log" 2>&1
  echo "Completed process/schema smoke check: $world_name (inspect logs for warnings)"
done
