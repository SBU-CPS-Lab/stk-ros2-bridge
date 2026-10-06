"""Which code produced a run: the git commit, and whether it was modified.

Recorded in every run record, run configuration and campaign manifest, so
a result can be traced to the exact bridge, scheduler and twin code that
produced it. Months later, "which version made this number" is otherwise
answered from memory.

The value is ``<40-hex commit>``, ``<commit>+dirty`` when tracked files
differ from that commit, or ``unknown (<reason>)``. A dirty or unknown
version is recorded as such, never omitted: a campaign run from modified
code is still data, but the paper must not attribute it to a commit.

Lookup order:

1. ``ACAV_CODE_VERSION`` in the environment. The run orchestrator sets it
   for the processes it starts, so the scheduler records the version the
   orchestrator resolved even when it runs from an install tree that is
   not a git checkout.
2. The git checkout containing this module.
3. The git checkout containing the current directory (the runbook runs
   every command from the workspace, inside the repository).
"""

from __future__ import annotations

import os
import subprocess
from typing import Optional

ENV_VAR = "ACAV_CODE_VERSION"


def _git(directory: str, *args: str) -> Optional[str]:
    try:
        completed = subprocess.run(
            ["git", "-C", directory, *args], capture_output=True,
            text=True, timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


def describe_checkout(directory: str) -> Optional[str]:
    """The version of the checkout containing ``directory``, or None."""
    commit = _git(directory, "rev-parse", "HEAD")
    if not commit:
        return None
    # Tracked files only: run outputs live under ignored directories, and
    # untracked local notes say nothing about the code that ran.
    changes = _git(directory, "status", "--porcelain", "--untracked-files=no")
    if changes is None:
        return f"{commit}+unknown-status"
    return f"{commit}+dirty" if changes else commit


def describe(prefer_env: bool = True) -> str:
    """The code version to record. Never raises.

    ``prefer_env=False`` is for the orchestrator, which must resolve the
    version from its own checkout rather than trust a value left in the
    operator's shell.
    """
    override = os.environ.get(ENV_VAR, "").strip()
    if prefer_env and override:
        return override
    here = os.path.dirname(os.path.realpath(__file__))
    for directory in (here, os.getcwd()):
        version = describe_checkout(directory)
        if version:
            return version
    return "unknown (no git checkout found)"


def is_clean(version: str) -> bool:
    """True for a plain commit hash: no local changes, nothing unknown."""
    return (len(version) == 40
            and all(c in "0123456789abcdef" for c in version))
