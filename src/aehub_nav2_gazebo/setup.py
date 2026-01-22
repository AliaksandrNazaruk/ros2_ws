from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aehub_nav2_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src', exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') if os.path.exists('launch') else []),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world') if os.path.exists('worlds') else []),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') if os.path.exists('urdf') else []),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml') if os.path.exists('params') else []),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*') if os.path.exists('maps') else []),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz') if os.path.exists('rviz') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='boris@example.com',
    description='Gazebo Classic Nav2 sim with MQTT bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
