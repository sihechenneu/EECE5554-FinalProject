#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Get the dynamic path to our custom package's share directory
    pkg_trash_detection = get_package_share_directory('trash_detection_2d')
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    
    # 2. Build absolute paths to our local configs
    oakd_yaml_path = os.path.join(pkg_trash_detection, 'config', 'oakd_yolo.yaml')
    yolo_json_path = os.path.join(pkg_trash_detection, 'config', 'yolov8.json')
    yolo_blob_path = os.path.join(pkg_trash_detection, 'config', 'yolov8n.blob')
    
    # 3. Read the YAML string, and dynamically inject the absolute config/blob paths into it!
    # Because `RewrittenYaml` in the turtlebot launch file doesn't support changing nested nn config paths
    # we rewrite a temporary YAML file on the fly with the correct absolute OS paths
    import tempfile
    import yaml
    
    with open(oakd_yaml_path, 'r') as f:
        config_data = yaml.safe_load(f)
        
    # Inject the absolute JSON path into the driver configuration
    config_data['/oakd']['ros__parameters']['nn']['i_nn_config_path'] = yolo_json_path
    
    # Write this modified config to a temporary file
    tmp_yaml = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(config_data, tmp_yaml)
    tmp_yaml.close()
    
    # We also need to dynamically rewrite the JSON file so the blob path is absolute
    with open(yolo_json_path, 'r') as f:
        json_data = yaml.safe_load(f) # json is a subset of yaml, safe_load handles it identically
    
    json_data['model']['model_name'] = yolo_blob_path
    
    tmp_json = tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False)
    import json
    json.dump(json_data, tmp_json, indent=4)
    tmp_json.close() # Flush and release the file lock so C++ can read it
    
    # NOW inject the modified temporary JSON path back into the temporary YAML!
    config_data['/oakd']['ros__parameters']['nn']['i_nn_config_path'] = tmp_json.name
    with open(tmp_yaml.name, 'w') as f:
        yaml.dump(config_data, f)
        
    print(f"Dynamically built OAK-D configurations for node launch!")
    print(f"  -> YAML: {tmp_yaml.name}")
    print(f"  -> JSON: {tmp_json.name}")
    print(f"  -> BLOB: {yolo_blob_path}")

    # 4. Include the original turtlebot4 oakd launch file, but point it to our generated custom parameters!
    oakd_launch_path = os.path.join(pkg_turtlebot4_bringup, 'launch', 'oakd.launch.py')
    
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(oakd_launch_path),
        launch_arguments={
            'params_file': tmp_yaml.name,
            'camera': 'oakd_pro'
        }.items()
    )
    
    return LaunchDescription([
        oakd_launch
    ])
