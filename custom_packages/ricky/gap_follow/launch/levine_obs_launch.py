from launch import LaunchDescription
from launch_ros.actions import Node

# levine_obs: obstacles and a slalom. Careful tuning.
# A long fixed look-ahead (4 m) lets the car see the L-shaped dead end in time.
EXECUTABLE = 'reactive_node'

PARAMS = {
    # Wall advisor. 'right' works in both directions on the Levine maps:
    # the inner wall clockwise, the outer wall counter-clockwise.
    'follow_wall': True,
    'wall_side': 'right',
    'wall_distance': 0.75,

    # Stop and back up a little when no ghost can move forward.
    'back_up': True,
    # Slow, precise THREAD mode for tight gaps.
    'use_thread': True,

    # The simulator's car steers up to 0.4189 rad. MEASURE the real car's
    # limit before using a launch file like this on it.
    'max_steering': 0.41,

    'max_speed': 2.0,
    'min_speed': 0.5,
    'side_margin': 0.2,      # room the car likes to keep on each side (m)
    'lookahead_time': 1.2,
    'lookahead_min': 4.0,
    'lookahead_max': 4.0,
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
