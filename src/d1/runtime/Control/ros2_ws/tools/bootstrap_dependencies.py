#!/usr/bin/env python3
"""Reproducible local apt extraction; never invokes sudo or changes /opt/ros."""
import subprocess
from pathlib import Path
ws = Path(__file__).resolve().parents[1]
root, debs = ws/'.deps/root', ws/'.deps/debs'
debs.mkdir(parents=True, exist_ok=True)
(ws/'.deps/COLCON_IGNORE').touch()
required = ['ros-jazzy-moveit-ros-move-group', 'ros-jazzy-moveit-kinematics',
            'ros-jazzy-moveit-planners-ompl', 'ros-jazzy-moveit-simple-controller-manager',
            'ros-jazzy-moveit-msgs', 'ros-jazzy-control-msgs']
result = subprocess.check_output(['apt-get','-s','install','--no-install-recommends',*required], text=True)
packages = sorted(set(required + [line.split()[1] for line in result.splitlines() if line.startswith('Inst ')]))
(ws/'.deps/packages.txt').write_text('\n'.join(packages)+'\n')
subprocess.run(['apt-get', 'download', *packages], cwd=debs, check=True)
for package in sorted(debs.glob('*.deb')):
    subprocess.run(['dpkg-deb', '-x', str(package), str(root)], check=True)
print('Dependencies extracted locally. Source env.sh then run ./run.sh build.')
