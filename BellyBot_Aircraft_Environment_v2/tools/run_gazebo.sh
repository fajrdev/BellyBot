#!/usr/bin/env bash
set -euo pipefail
asset_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world_name="${1:-transfer_resolved}"
world_file="$asset_root/ros2/bellybot_aircraft_environment/worlds/$world_name.sdf"
if [[ ! -f "$world_file" ]]; then echo "Unknown world: $world_name" >&2; exit 2; fi
export GZ_SIM_RESOURCE_PATH="$asset_root/ros2/bellybot_aircraft_environment/models:$asset_root/ros2${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -v 3 "$world_file"
