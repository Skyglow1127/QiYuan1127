from setuptools import setup
setup(name='vpr3_ros',version='0.2.0',packages=['vpr3_ros'],
    data_files=[('share/ament_index/resource_index/packages',['resource/vpr3_ros']),('share/vpr3_ros',['package.xml'])],
    entry_points={'console_scripts':['letterbox_node = vpr3_ros.node:main','verify_images = vpr3_ros.verify:main']})
