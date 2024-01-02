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
            # Timer
            "cup_noodle = ros2_example.timer.cup_noodle:main",
            "simple_timer = ros2_example.timer.simple_timer:main",
            "dual_timer = ros2_example.timer.dual_timer:main",
            "timer_wrrd = ros2_example.timer.timer_wrrd:main",
            "timer_wrrd_mt = ros2_example.timer.timer_wrrd_mt:main",
            # Topic
            "simple_pricing_publisher = ros2_example.topics.simple_pricing_publisher:main",
            "simple_pricing_subscriber = ros2_example.topics.simple_pricing_subscriber:main",
            "custom_pricing_publisher = ros2_example.topics.custom_pricing_publisher:main",
            "custom_pricing_subscriber = ros2_example.topics.custom_pricing_subscriber:main",
            # Service
            "circle_server = ros2_example.services.circle_server:main",
            "circle_client = ros2_example.services.circle_client:main",
            "price_server = ros2_example.services.price_server:main",
            "price_client_async = ros2_example.services.price_client_async:main",
            "price_client_async_callback = ros2_example.services.price_client_async_callback:main",
            # Action
            "sma_server = ros2_example.actions.sma_server:main",
            "sma_client_async = ros2_example.actions.sma_client_async:main",
            # Parameter
            "bb_param = ros2_example.params.bb_parameter:main",
        ],
    },
)
