#!/usr/bin/env bash
set -eo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
ws="$bundle_root/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
    exit 2
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is not installed" >&2
    exit 2
fi

source /opt/ros/humble/setup.bash
cd "$ws"
colcon build --base-paths "$ws/src" --executor sequential --symlink-install
source "$ws/install/local_setup.bash"
colcon test --base-paths "$ws/src" --executor sequential --event-handlers console_direct+
colcon test-result --verbose

for package_dir in "$ws/src"/*; do
    if [[ -d "$package_dir/test" ]]; then
        PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q "$package_dir/test"
    fi
done

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$bundle_root/runtime/phase_01_perception/src:$bundle_root" \
python3 -m pytest -q "$bundle_root/runtime/phase_01_perception/tests"

python3 "$bundle_root/scripts/validate_model.py"
echo "Software validation complete. This does not validate hardware or vehicle motion."

