from setuptools import setup

package_name = "reset"

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
    maintainer="Ian Rist",
    maintainer_email="ian@bigair.net",
    description="Reset sim",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reset = reset.node_reset:main",
        ],
    },
)
