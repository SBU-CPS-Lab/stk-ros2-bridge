"""Every path handed to MATLAB is absolute.

MATLAB runs from src/stk_matlab, so a relative stop-file or log path
given to `matlab -batch` would name another file there. MatlabBatchTwin
makes the run config, the log folder, the stop file and -logfile
absolute when it is built, and RunConfig.resolved_log_dir() is absolute.
"""

import os

from acav.twin_launcher import MatlabBatchTwin

from test_deploy_run_cli import _config  # noqa: I100



def test_the_matlab_command_has_only_absolute_paths(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    twin = MatlabBatchTwin(os.path.join("results", "X", "r.runconfig.json"),
                           os.path.join("results", "X", "r.logs"),
                           matlab_dir=str(tmp_path))
    expected_logs = os.path.join(str(tmp_path), "results", "X", "r.logs")
    assert twin.log_dir == expected_logs
    assert twin.stop_file == os.path.join(expected_logs, "twin.stop")
    assert twin.argv[-1] == os.path.join(expected_logs, "matlab.log")
    call = twin.argv[2]
    assert os.path.join(str(tmp_path), "results", "X",
                        "r.runconfig.json") in call
    assert os.path.join(expected_logs, "twin.stop") in call


def test_the_paths_stay_put_when_the_cwd_moves(tmp_path, monkeypatch):
    """Built in one folder, started from another (MATLAB's own cwd is
    src/stk_matlab): nothing follows the move."""
    monkeypatch.chdir(tmp_path)
    started = {}

    def popen(argv, cwd=None, **kwargs):
        started.update(argv=argv, cwd=cwd)
        return None

    twin = MatlabBatchTwin("r.runconfig.json", os.path.join("out", "r.logs"),
                           popen=popen,
                           matlab_dir=str(tmp_path))
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    twin.start()
    assert all(os.path.isabs(p) for p in (twin.log_dir, twin.stop_file,
                                          started["argv"][-1]))
    assert os.path.isdir(os.path.join(str(tmp_path), "out", "r.logs"))
    assert not (elsewhere / "out").exists()


def test_the_run_log_folder_is_absolute(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = _config(tmp_path, output=os.path.join("results", "X", "r.json"),
                     log_dir="")
    assert config.resolved_log_dir() == os.path.join(
        str(tmp_path), "results", "X", "r.logs")


def test_absolute_inputs_are_unchanged(tmp_path):
    logs = str(tmp_path / "r.logs")
    twin = MatlabBatchTwin(str(tmp_path / "c.json"), logs,
                           matlab_dir=str(tmp_path))
    assert twin.log_dir == logs
