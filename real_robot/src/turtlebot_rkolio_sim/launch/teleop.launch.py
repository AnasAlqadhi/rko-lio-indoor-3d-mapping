"""Launch teleop_twist_keyboard in a terminal window.

Prefers gnome-terminal, falls back to xterm or terminator. If none are
installed, prints the command for the user to run themselves.
"""

import shutil
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


def generate_launch_description():
    inner = (
        'source /opt/ros/humble/setup.bash && '
        'source ~/tb3_3d_ws/install/setup.bash && '
        'ros2 run teleop_twist_keyboard teleop_twist_keyboard'
    )

    if shutil.which('gnome-terminal'):
        cmd = ['gnome-terminal', '--', 'bash', '-c', inner]
    elif shutil.which('xterm'):
        cmd = ['xterm', '-e', 'bash', '-c', inner]
    elif shutil.which('terminator'):
        cmd = ['terminator', '-x', 'bash', '-c', inner]
    else:
        return LaunchDescription([
            LogInfo(msg=(
                'No terminal emulator found (tried gnome-terminal, xterm, terminator). '
                'Open a terminal yourself and run:\n  '
                'ros2 run teleop_twist_keyboard teleop_twist_keyboard'))
        ])

    return LaunchDescription([
        LogInfo(msg=f'Launching teleop in: {cmd[0]}'),
        ExecuteProcess(cmd=cmd, output='screen'),
    ])
