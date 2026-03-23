from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Arguments modifiables au lancement ---
    return LaunchDescription([

        DeclareLaunchArgument('x_min', default_value='0.0',   description='ROI X min (m)'),
        DeclareLaunchArgument('x_max', default_value='0.30',  description='ROI X max (m)'),
        DeclareLaunchArgument('y_min', default_value='-0.20', description='ROI Y min (m)'),
        DeclareLaunchArgument('y_max', default_value='0.1',   description='ROI Y max (m)'),
        DeclareLaunchArgument('decay_time',  default_value='5.0',  description='Fenêtre temporelle DBSCAN (s)'),
        DeclareLaunchArgument('eps',         default_value='0.006', description='Rayon voisinage DBSCAN (m)'),
        DeclareLaunchArgument('min_samples', default_value='50',   description='Points minimum par groupe'),


        # 2. Transform statique world -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_laser',
            arguments=['--frame-id', 'world', '--child-frame-id', 'laser'],
            output='screen',
        ),

        # 3. Node de filtrage ROI
        Node(
            package='ScanFilter',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[{
                'x_min': LaunchConfiguration('x_min'),
                'x_max': LaunchConfiguration('x_max'),
                'y_min': LaunchConfiguration('y_min'),
                'y_max': LaunchConfiguration('y_max'),
            }],
        ),

        # 4. Node d'analyse DBSCAN
        Node(
            package='ScanFilter',
            executable='scan_analysis_node',
            name='scan_analysis_node',
            output='screen',
            parameters=[{
                'decay_time':  LaunchConfiguration('decay_time'),
                'eps':         LaunchConfiguration('eps'),
                'min_samples': LaunchConfiguration('min_samples'),
            }],
        ),

        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

    ])
