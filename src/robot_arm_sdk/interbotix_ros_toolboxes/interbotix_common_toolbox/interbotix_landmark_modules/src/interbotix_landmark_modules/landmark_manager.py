#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from interbotix_landmark_modules.landmark import Landmark, LandmarkCollection


def main(args=None):
    rclpy.init(args=args)
    node = Node('landmark_manager')
    print("\n\nThis CLI application will be used to define and enable user-specified landmarks.")

    try:
        lm_path = get_package_share_directory('interbotix_landmark_modules')
    except Exception:
        lm_path = ''
    default_config = lm_path + '/landmarks/landmarks.yaml' if lm_path else 'landmarks.yaml'

    node.declare_parameter('landmark_config', default_config)
    node.declare_parameter('fixed_frame', 'landmarks')
    node.declare_parameter('obs_frame', 'camera_color_optical_frame')
    node.declare_parameter('standalone_tags', [
        {'id': 5, 'size': 0.02}, {'id': 413, 'size': 0.02}, {'id': 820, 'size': 0.02},
        {'id': 875, 'size': 0.02}, {'id': 1050, 'size': 0.02}])
    filepath = node.get_parameter('landmark_config').value
    fixed_frame = node.get_parameter('fixed_frame').value
    obs_frame = node.get_parameter('obs_frame').value
    param_tags = node.get_parameter('standalone_tags').value
    valid_tags = [tag['id'] for tag in param_tags]

    landmarks = LandmarkCollection(
        landmarks={},
        fixed_frame=fixed_frame,
        obs_frame=obs_frame,
        node=node)
    landmarks.load(filepath)
    cursor = "  >  "

    def get_input(type_):
        while True:
            try:
                return type_(input(cursor))
            except ValueError:
                print("Please enter a value of {}".format(type_))

    while True:
        print("Select landmark id you wish to create. Options: \n\t[ " + " ".join(str(t) for t in valid_tags) + " ]")
        while True:
            id_ = get_input(int)
            if id_ in valid_tags:
                break
            print("Please choose from the list of valid tags: {}".format(valid_tags))
        print("Landmark label (all lowercase).")
        label = get_input(str).lower()
        landmarks.add_landmark(label=label, id_num=id_)
        print("Is the tag mounted? Options: [y/n]")
        mounted_yn = get_input(str).lower()
        if mounted_yn in ("y", "yes"):
            mounted = True
            print("offset in meters: ")
            mounted_offset = get_input(float)
        else:
            mounted = False
            mounted_offset = 0.0
        landmarks.data[id_].mounted_ = mounted
        landmarks.data[id_].set_mounted_offset(mounted_offset)
        print("Enter another landmark? Options: [y/n]")
        more = get_input(str).lower()
        if more not in ("y", "yes"):
            print("Done adding landmarks. Saving and closing...\n\n")
            break

    landmarks.save(filepath)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
