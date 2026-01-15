from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aehub_navigation'

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
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools', 'requests', 'Pillow', 'numpy'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='boris@example.com',
    description='AE.HUB MVP Navigation - Nav2 Action Client, MQTT Adapter, Position Registry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_command_handler = aehub_navigation.navigation_command_handler:main',
            'mqtt_status_publisher = aehub_navigation.mqtt_status_publisher:main',
            'navigation_integrated_node = aehub_navigation.navigation_integrated_node:main',
        ],
    },
)

