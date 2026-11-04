from glob import glob
from setuptools import setup
setup(name='d1_pick_place', version='0.1.0', packages=['d1_pick_place'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/d1_pick_place']),
                  ('share/d1_pick_place', ['package.xml']),
                  ('share/d1_pick_place/config', glob('config/*')),
                  ('share/d1_pick_place/launch', glob('launch/*')),
                  ('share/d1_pick_place/meshes', glob('meshes/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['perception=d1_pick_place.perception:main',
                                        'hardware_bridge=d1_pick_place.hardware_bridge:main',
                                        'task=d1_pick_place.task:main',
                                        'offline=d1_pick_place.offline:main',
                                        'sim_state=d1_pick_place.sim_state:main']})
