from setuptools import setup

setup(
    name="trash_detection_2d",
    version="0.0.1",
    py_modules=["detector", "inference"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/trash_detection_2d"]),
        ("share/trash_detection_2d", ["package.xml"]),
    ],
    install_requires=["setuptools", "ultralytics"],
    entry_points={"console_scripts": ["trash_detection_node = inference:main_ros"]},
)
