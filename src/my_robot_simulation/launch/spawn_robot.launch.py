import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node # <-- WICHTIG: Schon vorhanden

def generate_launch_description():

    pkg_my_robot_simulation = get_package_share_directory('my_robot_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_my_robot_simulation, 'models')
    world_path = os.path.join(pkg_my_robot_simulation, 'worlds', 'car_world.sdf')
    bridge_config_path = os.path.join(pkg_my_robot_simulation, 'config', 'bridge.yaml')

    # 1. Umgebungsvariable für Modelle setzen (KORRIGIERT)
    # (Dein Block von vorher, der funktioniert)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    apriltag_models_path = '/home/david/gazebo_apriltag_harmonic/models'
    new_gz_path = f"{models_path}:{apriltag_models_path}:{existing_gz_path}"

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_path
    )

    # 2. Gazebo-Simulation starten
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-s -r ', world_path],
        }.items(),
    )

    # 3. Die Brücke
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )
    
    
    camera_info_node = Node(
    	package='my_robot_simulation',
    	executable='fake_camera_info_pub',
    	name='fake_camera_info_publisher'
	)


    # ===================================================================
    # === HIER KOMMT DEIN NEUER KNOTEN (aus dem ros2 run Befehl) ===
    # ===================================================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        # 'ros2 run ... -r' wird zu 'remappings'
        remappings=[
            ('/image_rect', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ],
        # 'ros2 run ... -p' wird zu 'parameters'
        # Wichtig: Parameter in Launch-Files sind eine Liste von Dictionaries
        parameters=[{
            'tag_family': 'tf36h11',
            'size': 0.20,  # <-- Passe das an, falls die Tag-Größe in der SDF anders ist!
            'publish_tag_detections_image': True
        }]
    )
    
    # ===================================================================
    # === WICHTIGE ERGÄNZUNG FÜR DEIN ZIEL (Position bestimmen) ===
    # ===================================================================
    # Du musst dem System sagen, wo das Tag in der Welt IST.
    # Annahme: Es ist bei X=2, Y=0, Z=0.5 und 90° um X gedreht (Roll=1.5708)
    static_tag_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tag_publisher',
        arguments=[
            # X, Y, Z
            '2.0', '0.0', '0.5',
            # Yaw, Pitch, Roll (in Radiant!)
            '0.0', '1.5708', '0.0',
            # Parent-Frame
            'world',
            # Child-Frame (Name des Tags)
            'tag36_11_00000' # <-- Stelle sicher, dass der Name zum Tag passt
        ]
    )

    return LaunchDescription([
        set_env,
        gz_sim,
        gz_bridge,
        apriltag_node,    
        static_tag_tf,     
        camera_info_node
    ])
