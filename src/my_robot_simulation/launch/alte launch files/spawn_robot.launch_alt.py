import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node # <-- WICHTIG: NEUER IMPORT

def generate_launch_description():

    pkg_my_robot_simulation = get_package_share_directory('my_robot_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    models_path = os.path.join(pkg_my_robot_simulation, 'models')
    world_path = os.path.join(pkg_my_robot_simulation, 'worlds', 'car_world.sdf')
    bridge_config_path = os.path.join(pkg_my_robot_simulation, 'config', 'bridge.yaml')

    # 1. Umgebungsvariable für Modelle setzen (KORRIGIERT)
    # Hole alle Pfade, die bereits existieren (z.B. von /opt/ros/jazzy/setup.bash)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # Definiere den Pfad zu deinen AprilTag-Modellen
    apriltag_models_path = '/home/david/gazebo_apriltag_harmonic/models'
    # Kombiniere alle Pfade: Dein Paket, die AprilTags und die bestehenden ROS-Pfade
    new_gz_path = f"{models_path}:{apriltag_models_path}:{existing_gz_path}"
    set_env = SetEnvironmentVariable(
    	name='GZ_SIM_RESOURCE_PATH',
    	value=new_gz_path
)

    # 2. Gazebo-Simulation starten (OHNE die Brücke)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -s für headless, -r für 'run'
            'gz_args': ['-s -r ', world_path],
            # 'bridge_config_file' wird entfernt, da wir es manuell starten
        }.items(),
    )

    # 3. Die Brücke EXPLIZIT als eigenen Knoten starten
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        gz_sim,
        gz_bridge  # <-- HIER DEN KNOTEN HINZUFÜGEN
    ])
