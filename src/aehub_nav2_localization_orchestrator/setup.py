from setuptools import setup, find_packages

package_name = "aehub_nav2_localization_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    package_dir={"": "src"},
    packages=find_packages(where="src", exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "requests"],
    zip_safe=True,
    maintainer="Boris",
    maintainer_email="boris@example.com",
    description="Localization bringup orchestrator: reload nav2_map_server and publish /initialpose after map updates.",
    license="Apache-2.0",
    tests_require=["pytest"],
)

