from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                # Wichtig: Family als Liste angeben!
                "tag_family": ["tf36h11"],
                # Größe deines Tags in Metern (anpassen!)
                "size": 0.04,
                # Falls du debug-images willst:
                "publish_tag_detections_image": True,
                # Optional: Filteroptionen
                "decimate": 1.0,
                "blur": 0.0,
                "threads": 2,
            }],
            remappings=[
                # Hier deine Kamera-Themen anpassen
                ("image_rect", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
                ("/tag_detections", "/detections"),
                ("tag_detections_image", "/tag_detections_image")
            ],
        )
    ])

