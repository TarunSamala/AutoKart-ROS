"""Keeps each test's ``sys.path``/``sys.modules`` state for the lazily-imported
``autokart_perception`` package isolated from other tests in this file/process.

Without this, a test that deliberately points ``algorithm_package_path`` at a directory with a
broken/partial ``autokart_perception`` package (to exercise the import-failure path) would poison
``sys.modules['autokart_perception']`` for the rest of the pytest process: Python caches the parent
package on first import, so a later test pointing at the real, complete package would still fail to
import its submodules, since the import machinery resolves the parent from the cache before
consulting ``sys.path`` again. This is a test-isolation concern only -- it does not affect the node at
runtime, which imports the dependency exactly once per process (see algorithm.py).
"""
import sys

import pytest


@pytest.fixture(autouse=True)
def _clean_autokart_perception_import_cache():
    original_path = list(sys.path)
    for name in list(sys.modules):
        if name == "autokart_perception" or name.startswith("autokart_perception."):
            del sys.modules[name]
    yield
    for name in list(sys.modules):
        if name == "autokart_perception" or name.startswith("autokart_perception."):
            del sys.modules[name]
    sys.path[:] = original_path
