from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from acav import ros_environment as ros


def test_missing_install_explains_build_requirement(tmp_path):
    with pytest.raises(RuntimeError, match='colcon build'):
        ros.workspace_environment(tmp_path)


def test_setup_loads_environment_without_changing_parent(tmp_path, monkeypatch):
    install=tmp_path/'workspace with spaces'/'install'
    install.mkdir(parents=True)
    (install/'setup.bat').write_text('@echo off')
    monkeypatch.setenv('PYTHONPATH','original')
    run=Mock(return_value=SimpleNamespace(returncode=0,stderr='',
        stdout='PYTHONPATH=overlay\nROS_DOMAIN_ID=42\nPath=overlay-dlls\n'))
    monkeypatch.setattr(ros.subprocess,'run',run)
    env=ros.workspace_environment(install.parent)
    assert env['PYTHONPATH']=='overlay'
    assert ros.os.environ['PYTHONPATH']=='original'
    assert env['ROS_DOMAIN_ID']=='42'
    assert env['PATH']=='overlay-dlls'
    assert sum(key.upper()=='PATH' for key in env)==1
    assert str(install/'setup.bat') in run.call_args.args[0]


def test_failed_type_support_never_launches_run(tmp_path,monkeypatch):
    monkeypatch.setattr(ros,'workspace_environment',lambda _: {})
    monkeypatch.setattr(ros.subprocess,'run',lambda *a,**k:
        SimpleNamespace(returncode=1,stderr='missing DLL'))
    launch=Mock()
    monkeypatch.setattr(ros.subprocess,'call',launch)
    with pytest.raises(RuntimeError,match='before starting the bridge'):
        ros.run_with_workspace('run.py',[],tmp_path)
    launch.assert_not_called()


def test_relaunch_uses_same_python_and_workspace_environment(tmp_path,monkeypatch):
    monkeypatch.setattr(ros,'workspace_environment',lambda _: {'ROS_DOMAIN_ID':'7'})
    monkeypatch.setattr(ros.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0))
    launch=Mock(return_value=0)
    monkeypatch.setattr(ros.subprocess,'call',launch)
    assert ros.run_with_workspace('run.py',['--seed','1'],tmp_path)==0
    assert launch.call_args.args[0]==[ros.sys.executable,'run.py','--seed','1']
    assert launch.call_args.kwargs['env']['ACAV_RUN_WORKSPACE']==str(tmp_path.resolve())
    assert launch.call_args.kwargs['env']['ROS_DOMAIN_ID']=='7'
