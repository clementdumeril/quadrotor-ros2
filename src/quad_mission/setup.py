from setuptools import setup

package_name = "quad_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Clément Dumeril",
    maintainer_email="clementdumeril@gmail.com",
    description="Mission layer (takeoff, land, go_to action, teleop) for the quadrotor stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission = quad_mission.mission:main",
            "square_demo = quad_mission.square_demo:main",
        ],
    },
)
