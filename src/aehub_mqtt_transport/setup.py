from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aehub_mqtt_transport'

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
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='boris@example.com',
    description='AE.HUB MQTT transport layer (ROS2 &lt;-&gt; MQTT)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mqtt_transport_node = aehub_mqtt_transport.mqtt_transport_node:main',
        ],
    },
)
