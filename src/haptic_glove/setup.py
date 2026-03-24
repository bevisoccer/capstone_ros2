from setuptools import setup

package_name = "haptic_glove"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bevisoccer",
    maintainer_email="bdacosta@andrew.cmu.edu",
    description="BLE haptic glove node",
    license="MIT",
    entry_points={
        "console_scripts": [
            "glove_node = haptic_glove.glove_node:main",
        ],
    },
)
