"""
Project configuration (config.yaml).

All tunable values of the project live in ``config.yaml`` in the project
folder.  Code reads them with::

    from quadrotor_controller.config import cfg
    mass = cfg("vehicle.mass")

Look-ups are strict: a missing key raises ``KeyError`` (there are no hidden
defaults in the code).  The file is chosen in this order:

1. ``load_config(path)`` / ``--config path`` on the runners,
2. the ``DRONES_CONFIG`` environment variable,
3. ``config.yaml`` next to this package.

PyYAML is used when installed; otherwise a small built-in parser handles the
subset used by config.yaml (nested maps, inline lists/maps, scalars, comments),
so the file also loads in minimal environments such as the Gazebo Python in WSL.
"""

from __future__ import annotations

import copy
import os
from typing import Any, Dict, Optional

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(PROJECT_ROOT, "config.yaml")
_MISSING = object()

_config: Optional[Dict[str, Any]] = None
_config_path: Optional[str] = None


# ---------------------------------------------------------------------------
# minimal YAML subset parser (fallback when PyYAML is not installed)
# ---------------------------------------------------------------------------

def _strip_comment(line: str) -> str:
    out, quote = [], None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(ch)
    return "".join(out).rstrip()


def _split_top(s: str, sep: str = ","):
    parts, depth, quote, cur = [], 0, None, []
    for ch in s:
        if quote:
            cur.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in "'\"":
            quote = ch
        elif ch in "[{":
            depth += 1
        elif ch in "]}":
            depth -= 1
        elif ch == sep and depth == 0:
            parts.append("".join(cur).strip())
            cur = []
            continue
        cur.append(ch)
    if "".join(cur).strip():
        parts.append("".join(cur).strip())
    return parts


def _scalar(s: str):
    s = s.strip()
    if s == "":
        return None
    if s[0] == "[" and s[-1] == "]":
        return [_scalar(p) for p in _split_top(s[1:-1])]
    if s[0] == "{" and s[-1] == "}":
        out = {}
        for p in _split_top(s[1:-1]):
            k, v = p.split(":", 1)
            out[k.strip()] = _scalar(v)
        return out
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "'\"":
        return s[1:-1]
    low = s.lower()
    if low in ("null", "~"):
        return None
    if low in ("true", "yes", "on"):
        return True
    if low in ("false", "no", "off"):
        return False
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


def parse_simple_yaml(text: str) -> Dict[str, Any]:
    root: Dict[str, Any] = {}
    stack = [(-1, root)]
    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        key, _, value = line.strip().partition(":")
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if value.strip() == "":
            child: Dict[str, Any] = {}
            parent[key.strip()] = child
            stack.append((indent, child))
        else:
            parent[key.strip()] = _scalar(value)
    return root


# ---------------------------------------------------------------------------
# public API
# ---------------------------------------------------------------------------

def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load (or reload) the configuration file and return it."""
    global _config, _config_path
    path = path or os.environ.get("DRONES_CONFIG") or DEFAULT_PATH
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(text)
    except ImportError:
        data = parse_simple_yaml(text)
    _config, _config_path = data or {}, os.path.abspath(path)
    return _config


def config_path() -> str:
    if _config is None:
        load_config()
    return _config_path


def cfg(key: str, default: Any = _MISSING) -> Any:
    """Value at a dotted key, e.g. cfg('controller.k_x').  Raises KeyError if absent
    (unless ``default`` is given explicitly by the caller)."""
    if _config is None:
        load_config()
    node: Any = _config
    for part in key.split("."):
        if isinstance(node, dict) and part in node:
            node = node[part]
        else:
            if default is not _MISSING:
                return default
            raise KeyError(f"'{key}' is missing in {_config_path} - add it to the configuration file")
    return copy.deepcopy(node)


def pick(value: Any, key: str) -> Any:
    """Use ``value`` if given, otherwise the configured value at ``key``."""
    return cfg(key) if value is None else value


def section(key: str, overrides: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """A configuration section (dict) merged with optional overrides."""
    out = dict(cfg(key))
    if overrides:
        out.update(overrides)
    return out


def project_path(key: str) -> str:
    """Absolute path of a file named in the 'paths' section."""
    p = cfg(f"paths.{key}")
    return p if os.path.isabs(p) else os.path.join(PROJECT_ROOT, p)


def set_value(key: str, value: Any) -> None:
    """Override one value at run time (used by --set key=value)."""
    if _config is None:
        load_config()
    node = _config
    parts = key.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = _scalar(value) if isinstance(value, str) else value


def banner(title: str = "") -> str:
    """Console banner of logging.banner_width characters."""
    line = "=" * cfg("logging.banner_width")
    return f"{line}\n  {title}\n{line}" if title else line


def add_config_arguments(parser) -> None:
    """Adds --config and --set to an argparse parser."""
    parser.add_argument("--config", type=str, default=None, help="configuration file (default: config.yaml)")
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="override a configuration value, e.g. --set controller.k_x=1.5")


def apply_config_arguments(args) -> None:
    load_config(getattr(args, "config", None))
    for item in getattr(args, "set", []) or []:
        k, _, v = item.partition("=")
        set_value(k.strip(), v.strip())
