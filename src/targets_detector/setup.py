from setuptools import find_packages, setup

PACKAGE_NAME = "targets_detector"

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
    description="Simulated vision of the ball-sorting demo: publishes balls resting on the table in the work zone.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "targets_detector = targets_detector.detector_node:main",
        ],
    },
)
