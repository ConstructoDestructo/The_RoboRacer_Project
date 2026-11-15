from launch import LaunchDescription
from launch_ros.actions import Node

# levine_obs: obstacles and a slalom. Careful tuning.
EXECUTABLE = 'reactive_node'

PARAMS = {
    # Wall advisor. Clockwise laps: the inner wall is on the car's RIGHT.
    # For counter-clockwise laps set wall_side to 'left'.
    'follow_wall': True,
    'wall_side': 'right',
    'wall_distance': 0.75,

    # Stop and back up a little when no ghost can move forward.
    'back_up': True,

    'max_speed': 2.0,
    'min_speed': 0.5,
    'side_margin': 0.2,      # room the car likes to keep on each side (m)
    'lookahead_time': 1.2,
    'lookahead_min': 1.8,
    'lookahead_max': 3.5,
    'lateral_accel': 4.0,
    'brake_decel': 4.0,
}


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable=EXECUTABLE,
            name='reactive_node',
            output='screen',
            parameters=[PARAMS],
        ),
    ])
