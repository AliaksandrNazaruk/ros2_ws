from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aehub_navigation_executor'

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
        (os.path.join('share', package_name, 'systemd'),
            glob('systemd/*.service.example') if os.path.exists('systemd') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='boris@example.com',
    description='Business-layer Nav2 command executor with idempotency',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_executor_node = aehub_navigation_executor.navigation_executor_node:main',
            'sim_robot_status_publisher = aehub_navigation_executor.sim_robot_status_publisher:main',
        ],
    },
)
