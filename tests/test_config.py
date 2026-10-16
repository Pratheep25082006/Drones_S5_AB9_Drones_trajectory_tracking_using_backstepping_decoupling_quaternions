"""Tests for config.yaml loading (no hard-coded values: everything comes from the config file)."""
import ast
import glob
import os
import sys
import tempfile

import pytest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from quadrotor_controller import config  # noqa: E402
from quadrotor_controller.config import cfg, load_config, parse_simple_yaml, set_value  # noqa: E402


@pytest.fixture(autouse=True)
def _reload_default():
    load_config()
    yield
    load_config()


def test_fallback_parser_matches_pyyaml():
    yaml = pytest.importorskip("yaml")
    text = open(os.path.join(ROOT, "config.yaml"), encoding="utf-8").read()
    assert parse_simple_yaml(text) == yaml.safe_load(text)


def test_missing_key_raises_clear_error():
    with pytest.raises(KeyError, match="missing"):
        cfg("controller.does_not_exist")


def test_override_with_set_and_other_file():
    set_value("controller.k_x", "1.5")
    assert cfg("controller.k_x") == 1.5
    from quadrotor_controller import PaperBacksteppingController
    assert PaperBacksteppingController().gains.k_x == 1.5
    with tempfile.TemporaryDirectory() as d:
        alt = os.path.join(d, "alt.yaml")
        text = open(os.path.join(ROOT, "config.yaml"), encoding="utf-8").read().replace("  mass: 1.5", "  mass: 2.0", 1)
        open(alt, "w", encoding="utf-8").write(text)
        load_config(alt)
        assert cfg("vehicle.mass") == 2.0
        assert PaperBacksteppingController().mass == 2.0


def test_paper_gains_are_the_configured_defaults():
    from quadrotor_controller import PaperGains
    g = PaperGains()
    for k in ("k_x", "k_v", "k_t", "k_r", "k_phi", "k_wz"):
        assert getattr(g, k) == cfg(f"controller.{k}")


def test_no_tunable_numbers_in_code():
    """Guards against new hard-coded values: only 0..6, 0.5 and +-1 may appear as literals
    (structure / formulas); everything else must come from config.yaml or units.py."""
    allowed = {0, 1, 2, 3, 4, 5, 6, -1, 0.5}
    exempt = {"units.py"}
    formula_files = {"sitl_bridge.py": {0.25}, "track_error.py": {7, 8, 9, 10, 11, 12, 13}}
    offenders = []
    for f in glob.glob(os.path.join(ROOT, "quadrotor_controller", "*.py")) + glob.glob(os.path.join(ROOT, "*.py")):
        name = os.path.basename(f)
        if name in exempt:
            continue
        ok = allowed | formula_files.get(name, set())
        for n in ast.walk(ast.parse(open(f, encoding="utf-8").read())):
            if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool):
                if n.value not in ok:
                    offenders.append(f"{name}:{n.lineno} {n.value!r}")
    assert not offenders, "hard-coded values found:\n" + "\n".join(offenders)
