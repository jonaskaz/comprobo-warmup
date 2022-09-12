from setuptools import setup

package_name = "comprobo_warmup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="katz",
    maintainer_email="jonas.kazlauskas@me.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "marker = comprobo_warmup.marker:main",
            "teleop = comprobot_warmup.teleop:main"
        ],
    },
)
