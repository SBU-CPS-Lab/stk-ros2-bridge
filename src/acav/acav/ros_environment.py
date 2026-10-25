"""Load a built Windows ROS workspace into child processes only."""
import os
from pathlib import Path
import subprocess
import sys


def workspace_environment(workspace):
    setup = Path(workspace).resolve() / 'install' / 'setup.bat'
    if not setup.is_file():
        raise RuntimeError(f'Workspace is not built: {setup} is missing. Run colcon build first.')
    result = subprocess.run(
        f'cmd.exe /d /s /c "call "{setup}" >nul && set"',
        capture_output=True, text=True, errors='replace', timeout=60)
    if result.returncode:
        raise RuntimeError(f'Cannot load ROS workspace: {result.stderr.strip()}')
    # Windows environment names are case-insensitive. A plain dict is not:
    # retaining both PATH and Path lets the child select the old DLL path.
    env = {key.upper(): value for key, value in os.environ.items()}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition('=')
        if separator and key and not key.startswith('='):
            env[key.upper()] = value
    return env


def run_with_workspace(script, args, workspace):
    """Restart once so Python, DLL search, and all children share the overlay."""
    env = workspace_environment(workspace)
    check = subprocess.run(
        [sys.executable, '-c',
         'import rclpy; from stk_msgs.msg import BridgeStatus; '
         'rclpy.init(); node=rclpy.create_node("acav_type_support_probe"); '
         'sub=node.create_subscription(BridgeStatus,"/acav_type_support_probe",lambda m:None,10); '
         'node.destroy_node(); rclpy.shutdown()'],
        env=env, capture_output=True, text=True, timeout=30)
    if check.returncode:
        raise RuntimeError('ROS subscription/type support check failed before starting the bridge. '
                           'Check the ROS Python version and rebuild stk_msgs.\n' + check.stderr)
    env['ACAV_RUN_WORKSPACE'] = str(Path(workspace).resolve())
    return subprocess.call([sys.executable, str(script), *args], env=env)
