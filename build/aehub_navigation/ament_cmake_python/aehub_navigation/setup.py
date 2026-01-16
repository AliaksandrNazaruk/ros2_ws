from setuptools import find_packages
from setuptools import setup

setup(
    name='aehub_navigation',
    version='0.1.0',
    packages=find_packages(
        include=('aehub_navigation', 'aehub_navigation.*')),
)
