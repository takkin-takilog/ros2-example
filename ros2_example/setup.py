from setuptools import find_packages, setup

package_name = "ros2_example"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takkin",
    maintainer_email="takkin.takilog@gmail.com",
    description="ros2_example package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simple_pricing_publisher = ros2_example.topics.simple_pricing_publisher:main",
            "simple_pricing_subscriber = ros2_example.topics.simple_pricing_subscriber:main",
        ],
    },
)
