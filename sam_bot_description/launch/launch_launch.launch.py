from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    display_lauch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_bot_description'),
                         'launch/display.launch.py')
        )
    )

    navigation_lauch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_bot_description'),
                         'launch/navigation_launch.py')
        )
    )

    trasform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_bot_description'),
                         'launch/static_transform.py')
        )
    )

    ld.add_action(display_lauch)
    ld.add_action(navigation_lauch)
    ld.add_action(trasform_launch)
    return ld
