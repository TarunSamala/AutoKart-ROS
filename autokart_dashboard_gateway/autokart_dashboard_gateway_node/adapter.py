"""Lazy-import adapter around the existing, already-tested dashboard gateway
(``kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway``:
``gateway.py``, ``core.py``, ``camera_feed.py``).

This package's own python module is deliberately named
``autokart_dashboard_gateway_node`` -- NOT ``autokart_dashboard_gateway`` --
even though its ROS package/directory name (and this library's "node 11")
IS ``autokart_dashboard_gateway``. The dependency being reused is itself a
Python package also named ``autokart_dashboard_gateway``
(``kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/``).
Giving this wrapper's importable module the same name as the thing it
imports would make ``import autokart_dashboard_gateway.gateway`` ambiguous
with (or recurse into) this very package. Using a distinct inner module name
sidesteps that collision entirely while keeping the ROS package name the
task calls for. See DEPENDENCIES.md.

Nothing in ``gateway.py``/``core.py``/``camera_feed.py`` is reimplemented
here -- this adapter only resolves an import path and re-exports what it
loads.
"""
from __future__ import annotations

import importlib
import sys
from pathlib import Path
from types import ModuleType
from typing import Optional


class GatewayAdapterError(RuntimeError):
    """Raised for any adapter fault: missing dependency path, import failure,
    or an import that does not expose the expected API."""


REQUIRED_GATEWAY_ATTRS = ("make_ros_node", "create_app", "load_replay", "ALLOWED_SAFETY_REQUESTS")
REQUIRED_CORE_ATTRS = ("MissionStore", "valid_frame")


def _reset_module_cache(top_level_name: str) -> None:
    for name in list(sys.modules):
        if name == top_level_name or name.startswith(top_level_name + "."):
            del sys.modules[name]


def _move_to_front(src_str: str) -> None:
    while src_str in sys.path:
        sys.path.remove(src_str)
    sys.path.insert(0, src_str)


def load_gateway(package_path: str, perception_package_path: Optional[str] = None) -> ModuleType:
    """Resolve and import the real ``autokart_dashboard_gateway.gateway`` module.

    ``package_path`` must be the ROS package directory that CONTAINS the
    nested ``autokart_dashboard_gateway`` Python package, e.g.
    ``kart/ros2_ws/src/autokart_dashboard_gateway`` -- never hardcoded here.
    See ``autokart_safety.adapter.load_safety_core`` for the identical
    pattern and the path-shadowing pitfall this function also guards
    against (always moving the resolved path to the front of ``sys.path``
    and purging any stale cached module under the same dotted name first).

    IMPORTANT, discovered while building this adapter: ``gateway.py``
    unconditionally does ``from .camera_feed import CameraFeed`` at module
    level, and ``camera_feed.py`` unconditionally does
    ``from autokart_perception.live_render import depth_preview,
    encode_jpeg``. Importing ``autokart_dashboard_gateway.gateway`` therefore
    ALWAYS requires the separate ``autokart_perception`` package to be
    importable too -- even when camera display is disabled
    (``enable_camera_feed=false``) and even in replay-only mode. This is a
    property of the existing, reused code (not something this adapter
    introduces or could lazily defer without editing that file, which is out
    of scope) -- see DEPENDENCIES.md. ``perception_package_path`` is where
    this adapter resolves that transitive dependency from.
    """
    if not package_path:
        raise GatewayAdapterError("DEPENDENCY_PATH_NOT_SET")
    src = Path(package_path).expanduser()
    if not src.is_dir():
        raise GatewayAdapterError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
    if not perception_package_path:
        raise GatewayAdapterError("PERCEPTION_DEPENDENCY_PATH_NOT_SET")
    perception_src = Path(perception_package_path).expanduser()
    if not perception_src.is_dir():
        raise GatewayAdapterError(f"PERCEPTION_DEPENDENCY_PATH_NOT_FOUND:{perception_src}")
    _move_to_front(str(src))
    _move_to_front(str(perception_src))
    _reset_module_cache("autokart_dashboard_gateway")
    _reset_module_cache("autokart_perception")
    # Explicitly resolve autokart_perception FIRST and verify where it
    # actually came from, before letting gateway.py's own transitive import
    # pull it in implicitly. This machine (and potentially others) has
    # autokart_perception permanently importable via a site-wide editable
    # install (an __editable__.autokart_perception*.pth in site-packages),
    # independent of perception_package_path entirely. Without this check, a
    # WRONG perception_package_path would not fail as this adapter's contract
    # promises -- it would silently succeed by falling through to whatever
    # globally-installed copy happens to be on sys.path, which may not be the
    # version the deployer intended. Found while building this adapter's own
    # tests; see test_adapter.py's wrong-perception-path regression test.
    try:
        perception = importlib.import_module("autokart_perception")
    except Exception as exc:  # noqa: BLE001
        raise GatewayAdapterError(
            f"GATEWAY_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    resolved = Path(getattr(perception, "__file__", "") or "").resolve()
    try:
        resolved.relative_to(perception_src.resolve())
    except ValueError:
        raise GatewayAdapterError(
            f"PERCEPTION_DEPENDENCY_PATH_MISMATCH:requested={perception_src} resolved={resolved} "
            "(a different autokart_perception is importable elsewhere on sys.path -- likely a "
            "site-wide editable install -- and would otherwise be silently used instead)")
    try:
        gateway = importlib.import_module("autokart_dashboard_gateway.gateway")
    except Exception as exc:  # noqa: BLE001 - deliberate: surface as a typed fault
        raise GatewayAdapterError(f"GATEWAY_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    missing = [name for name in REQUIRED_GATEWAY_ATTRS if not hasattr(gateway, name)]
    if missing:
        raise GatewayAdapterError(f"GATEWAY_API_MISMATCH:missing={','.join(missing)}")
    return gateway


def load_core(package_path: str) -> ModuleType:
    """Resolve and import the real ``autokart_dashboard_gateway.core`` module
    (``MissionStore``, ``valid_frame``). Pure Python, no ROS/aiohttp
    dependency -- this is what makes the "replay"/synthetic profile of this
    node buildable and runnable with zero ROS graph."""
    if not package_path:
        raise GatewayAdapterError("DEPENDENCY_PATH_NOT_SET")
    src = Path(package_path).expanduser()
    if not src.is_dir():
        raise GatewayAdapterError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
    _move_to_front(str(src))
    _reset_module_cache("autokart_dashboard_gateway")
    try:
        core = importlib.import_module("autokart_dashboard_gateway.core")
    except Exception as exc:  # noqa: BLE001
        raise GatewayAdapterError(f"GATEWAY_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    missing = [name for name in REQUIRED_CORE_ATTRS if not hasattr(core, name)]
    if missing:
        raise GatewayAdapterError(f"GATEWAY_API_MISMATCH:missing={','.join(missing)}")
    return core


def load_camera_feed(package_path: str) -> ModuleType:
    """Resolve and import the real ``autokart_dashboard_gateway.camera_feed``
    module. Requires the separate ``autokart_perception`` dependency
    (``live_render``) to actually be importable -- this is why camera
    support is optional (``enable_camera_feed`` parameter, default false) and
    never silently assumed available. See DEPENDENCIES.md."""
    if not package_path:
        raise GatewayAdapterError("DEPENDENCY_PATH_NOT_SET")
    src = Path(package_path).expanduser()
    if not src.is_dir():
        raise GatewayAdapterError(f"DEPENDENCY_PATH_NOT_FOUND:{src}")
    _move_to_front(str(src))
    _reset_module_cache("autokart_dashboard_gateway")
    try:
        camera_feed = importlib.import_module("autokart_dashboard_gateway.camera_feed")
    except Exception as exc:  # noqa: BLE001
        raise GatewayAdapterError(f"CAMERA_FEED_IMPORT_FAILED:{type(exc).__name__}:{exc}") from exc
    if not hasattr(camera_feed, "CameraFeed"):
        raise GatewayAdapterError("CAMERA_FEED_API_MISMATCH:missing=CameraFeed")
    return camera_feed
