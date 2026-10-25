"""Which run config the twin loaded, and which code it runs.

The Simulink model is built with its run-config path baked in
(build_acav_twin.m), so pressing Play on the previous run's model loads
the previous run's config without any error. Once ready, the twin
publishes on IDENTITY_TOPIC (std_msgs/String):

    v3 <cfg> <build> <runtime> <polling>

``cfg``      sha256 of the run-config bytes it parsed (``-`` for none)
``build``    code fingerprint written into the model when it was built
``runtime``  code fingerprint recomputed from disk at simulation start
``polling``  the model's step rate per wall second (Simulink pacing,
             build_acav_twin's PollingRateHz), e.g. ``100Hz``; ``-`` if
             not given

each fingerprint truncated to FINGERPRINT_CHARS hex digits, so the
message is 107 bytes at 100 Hz. The twin publishes it on its first
ready step and then about once per wall second. The run orchestrator
checks it before starting the scheduler (deploy_run_cli.py), and the
record keeps the text (Provenance.twin_identity).

The fingerprint is a sha256 over ``<path>\\0<sha256 of the file>\\n``
lines, sorted by path, for the twin's MATLAB code, its Simulink build
script, the scenario YAML it reads and the message definitions its bus
types come from. +adcs/code_fingerprint.m computes the same; both test
suites pin one vector. Each file's text is hashed with CRLF turned into
LF, so the line endings a checkout uses (core.autocrlf) do not change it.

Format versions: v1 had no polling rate; v2 added it; v3 marks the
line-ending-free fingerprint. The runner accepts only v3.
"""

from __future__ import annotations

import glob
import hashlib
import os
import re
from dataclasses import dataclass
from typing import List, Optional

IDENTITY_TOPIC = "/acav/twin_identity"
FORMAT_VERSION = "v3"
FINGERPRINT_CHARS = 16
#: Room the twin has for the message (build_acav_twin.m checks it).
MAX_IDENTITY_BYTES = 128

#: Relative to the workspace's src directory, '/'-separated.
_PATTERNS = ("stk_matlab/+adcs/**/*.m",
             "stk_matlab/simulink/build_acav_twin.m",
             "acav/config/acav_scenario.yaml",
             "stk_msgs/msg/*.msg")
_REQUIRED = ("stk_matlab/simulink/build_acav_twin.m",
             "acav/config/acav_scenario.yaml")

_HEX64 = re.compile(r"[0-9a-f]{64}")
_HEX_FP = re.compile(r"[0-9a-f]{%d}" % FINGERPRINT_CHARS)
_POLLING = re.compile(r"([0-9]+(?:\.[0-9]+)?)Hz")


def source_root() -> str:
    """The src directory of the checkout this module belongs to."""
    return os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))


def fingerprint_files(src_root: str) -> List[str]:
    """The files the fingerprint covers, as sorted relative paths."""
    found = set()
    for pattern in _PATTERNS:
        for path in glob.glob(os.path.join(src_root, *pattern.split("/")),
                              recursive=True):
            if os.path.isfile(path):
                found.add(os.path.relpath(path, src_root).replace(os.sep,
                                                                  "/"))
    for required in _REQUIRED:
        if required not in found:
            raise FileNotFoundError(
                f"{required} not found under {src_root}")
    return sorted(found)


def file_sha256(path: str) -> str:
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def text_sha256(path: str) -> str:
    """sha256 of a file's text with CRLF turned into LF (a lone CR kept)."""
    with open(path, "rb") as handle:
        return hashlib.sha256(
            handle.read().replace(b"\r\n", b"\n")).hexdigest()


def code_fingerprint(src_root: str) -> str:
    lines = b"".join(
        relative.encode("utf-8") + b"\0"
        + text_sha256(os.path.join(src_root, *relative.split("/"))).encode()
        + b"\n"
        for relative in fingerprint_files(src_root))
    return hashlib.sha256(lines).hexdigest()


@dataclass(frozen=True)
class Identity:
    config_sha256: str
    build_fingerprint: str
    runtime_fingerprint: str
    #: The model's polling rate, Hz; None when the twin gave none.
    polling_hz: Optional[float] = None


def parse_identity(text: str, versions=(FORMAT_VERSION,)) -> Identity:
    """The identity in ``text``. Only the current format by default (the
    runner's check); an analysis of older records may pass ("v2", "v3"),
    which share the layout."""
    parts = text.split(" ")
    if len(parts) != 5 or parts[0] not in versions:
        raise ValueError(f"expected '{FORMAT_VERSION} <cfg> <build> "
                         f"<runtime> <polling>', got {text!r}")
    config, build, runtime, polling = parts[1:]
    if config != "-" and not _HEX64.fullmatch(config):
        raise ValueError(f"bad run-config digest {config!r}")
    for value in (build, runtime):
        if not _HEX_FP.fullmatch(value):
            raise ValueError(f"bad code fingerprint {value!r}")
    polling_hz = None
    if polling != "-":
        match = _POLLING.fullmatch(polling)
        if not match or float(match.group(1)) <= 0.0:
            raise ValueError(f"bad polling rate {polling!r}")
        polling_hz = float(match.group(1))
    return Identity(config, build, runtime, polling_hz)


def check_identity(text: str, run_config: str, src_root: str) -> List[str]:
    """Why this twin must not run this run; empty when it may."""
    try:
        identity = parse_identity(text)
    except ValueError as error:
        return [f"the twin identity is not readable: {error}"]
    problems = []
    try:
        expected_config = file_sha256(run_config)
    except OSError as error:
        return [f"cannot read this run's configuration {run_config}: "
                f"{error}"]
    if identity.config_sha256 == "-":
        problems.append(
            "the twin loaded no run configuration; this run's is "
            f"{run_config}. Rebuild with build_acav_twin('RunConfigPath', "
            f"...)")
    elif identity.config_sha256 != expected_config:
        problems.append(
            f"the twin loaded a different run configuration (sha256 "
            f"{identity.config_sha256}); this run's {run_config} is "
            f"{expected_config}. Was the previous run's model left open?")
    if identity.build_fingerprint != identity.runtime_fingerprint:
        problems.append(
            f"the Simulink model was built from other code (build "
            f"{identity.build_fingerprint}, code on disk "
            f"{identity.runtime_fingerprint}); rebuild it")
    expected_code = code_fingerprint(src_root)[:FINGERPRINT_CHARS]
    if identity.runtime_fingerprint != expected_code:
        problems.append(
            f"the twin runs MATLAB code that differs from this checkout "
            f"(twin {identity.runtime_fingerprint}, checkout "
            f"{expected_code}); start MATLAB from {src_root}")
    return problems
