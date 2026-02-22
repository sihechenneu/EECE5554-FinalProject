import os
from glob import glob
from setuptools import find_packages, setup

package_name = "trash_detection_2d"
setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/trash_detection_2d"]),
        ("share/trash_detection_2d", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "ultralytics"],
    entry_points={
        "console_scripts": [
            "yolo_trash_publisher = yolo_trash.yolo_trash_publisher:main",
            "yolo_trash_subscriber = yolo_trash.yolo_trash_subscriber:main",
            "yolo_trash_inference = yolo_trash.inference:main",
        ],
    },
)
