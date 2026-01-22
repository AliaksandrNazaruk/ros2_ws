from setuptools import setup, find_packages
import os

package_name = "aehub_symovo_map_mirror"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "requests", "Pillow", "numpy"],
    zip_safe=True,
    maintainer="Boris",
    maintainer_email="boris@example.com",
    description="Symovo map mirror (ROS interface): fetch/refresh map artifacts + publish typed map status.",
    license="Apache-2.0",
    tests_require=["pytest"],
)

