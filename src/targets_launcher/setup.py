from setuptools import find_packages, setup

PACKAGE_NAME = "targets_launcher"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Course Instructor",
    maintainer_email="instructor@example.com",
    description="Helper node of the ball-sorting demo: feeds balls from the bins into the work zone.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "targets_launcher = targets_launcher.launcher_node:main",
        ],
    },
)
