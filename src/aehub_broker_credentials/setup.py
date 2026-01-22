from glob import glob

import os

from setuptools import find_packages, setup

package_name = 'aehub_broker_credentials'

setup(
    name=package_name,
    version='0.1.0',
    package_dir={'': 'src'},
    packages=find_packages(where='src', exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') if os.path.exists('launch') else [],
        ),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Boris',
    maintainer_email='boris@example.com',
    description=(
        'AE.HUB Broker Credentials - Lifecycle node for fetching and publishing MQTT broker '
        'configuration'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker_credentials_node = aehub_broker_credentials.broker_credentials_node:main',
        ],
    },
)
