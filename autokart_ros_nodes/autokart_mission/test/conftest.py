"""Keeps each test's ``sys.path``/``sys.modules`` state for the lazily-imported
``autokart_perception``, ``autokart_vehicle_geometry`` and ``autokart_planner`` packages isolated
from other tests in this file/process. See autokart_mapping/test/conftest.py for the full rationale
-- the same hazard applies here, for all three lazily-imported package names instead of one.
"""
import sys

import pytest

_PREFIXES = ("autokart_perception", "autokart_vehicle_geometry", "autokart_planner")


def _purge():
    for name in list(sys.modules):
        if any(name == prefix or name.startswith(prefix + ".") for prefix in _PREFIXES):
            del sys.modules[name]


@pytest.fixture(autouse=True)
def _clean_lazy_import_cache():
    original_path = list(sys.path)
    _purge()
    yield
    _purge()
    sys.path[:] = original_path
