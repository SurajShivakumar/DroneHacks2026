"""Declare optional local experiment inputs without hiding assertion failures."""

from pathlib import Path

import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--require-local-artifacts", action="store_true", default=False,
        help="Fail instead of skipping tests whose declared local inputs are missing.",
    )


def pytest_runtest_setup(item):
    root = Path(__file__).resolve().parents[1]
    for marker in item.iter_markers(name="local_artifacts"):
        missing = [name for name in marker.args if not (root / name).exists()]
        if missing:
            reason = "Missing local experiment inputs: " + ", ".join(missing)
            if item.config.getoption("--require-local-artifacts"):
                pytest.fail(reason, pytrace=False)
            pytest.skip(reason + "; restore inputs to run this integration check")
