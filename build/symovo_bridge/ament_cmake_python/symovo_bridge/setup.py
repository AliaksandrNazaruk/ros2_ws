from setuptools import find_packages
from setuptools import setup

setup(
    name='symovo_bridge',
    version='0.1.0',
    packages=find_packages(
        include=('symovo_bridge', 'symovo_bridge.*')),
)
