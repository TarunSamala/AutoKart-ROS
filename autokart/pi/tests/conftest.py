import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ("perception", "mapping", "planner", "controller", "vehicle_bridge", "safety"):
    sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", f"autokart_{pkg}"))
sys.path.insert(0, os.path.join(ROOT, "dashboard", "ros_gateway"))
