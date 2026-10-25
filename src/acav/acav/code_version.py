"""Which code produced a run: the git commit, and whether it was modified.

Recorded in every run record, run configuration and campaign manifest, so
a result can be traced to the exact bridge, scheduler and twin code that
produced it.

The value is ``<40-hex commit>``, ``<commit>+dirty`` when tracked files
differ from that commit, or ``unknown (<reason>)``. A dirty or unknown
version is recorded as such, never omitted: a campaign run from modified
code is still data, but it cannot be attributed to a commit.

Lookup order:

1. ``ACAV_CODE_VERSION`` in the environment. The run orchestrator sets it
   for the processes it starts, so the scheduler records the version the
   orchestrator resolved even when it runs from an install tree that is
   not a git checkout.
2. The git checkout containing this module.
3. The git checkout containing the current directory.
"""

from __future__ import annotations

import os
import subprocess
from typing import Callable, Optional

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

    ``prefer_env=False`` is for the orchestrator, which resolves the
    version from its own checkout rather than trusting a value left in
    the shell.
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


def tree_of(commit: str, directory: str = "") -> Optional[str]:
    """The git tree of ``commit``, or None if it cannot be resolved."""
    directory = directory or os.path.dirname(os.path.realpath(__file__))
    return _git(directory, "rev-parse", "--verify", "--quiet",
                f"{commit}^{{tree}}") or None


def same_code(recorded: str, current: str,
              tree_of: Callable[[str], Optional[str]] = tree_of) -> bool:
    """Whether two recorded versions name the same code.

    Both must be clean commits. Equal hashes are the same code; different
    hashes are the same code only if they resolve to the same git tree,
    which is what a message-only history rewrite leaves behind.
    """
    if not (is_clean(recorded) and is_clean(current)):
        return False
    if recorded == current:
        return True
    tree = tree_of(recorded)
    return tree is not None and tree == tree_of(current)
