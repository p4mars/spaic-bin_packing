"""
mission.launch.py — ONE launch for the full mission on the REAL robot.

Brings up the shuttle stack (SLAM + Nav2 + shuttle_manager) with all the
real-robot args baked in (DICT_4X4_250, A=100, B=101/102, 8 cm printed markers,
dock_at_b, the real cmd_vel, compressed camera).  At Zone B shuttle_manager
SPAWNS the precision team's marker_navigator.py (precise dock between 101/102)
and box_placer.py (lay-down → walk-back → return-home; no box is actually
grabbed), then resumes to A on /robot_backed_up.  Their scripts are unchanged —
configured purely via params/remaps.

    # detection ON THE ROBOT (default):
    ros2 launch mirte_driving mission.launch.py

    # detection OFFLOADED TO THE LAPTOP (run detector.launch.py there):
    ros2 launch mirte_driving mission.launch.py run_zone_detector:=false
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    run_zone_detector = LaunchConfiguration('run_zone_detector')
    dock_wait_for_box = LaunchConfiguration('dock_wait_for_box')
    use_depth_scan    = LaunchConfiguration('use_depth_scan')
    grasp_at_a        = LaunchConfiguration('grasp_at_a')
    dock_at_b         = LaunchConfiguration('dock_at_b')
    arm_mimic         = LaunchConfiguration('arm_mimic')
    round_trips       = LaunchConfiguration('round_trips')
    udp_only          = LaunchConfiguration('udp_only')
    scan_min_range    = LaunchConfiguration('scan_min_range')
    arm_up_angles     = LaunchConfiguration('arm_up_angles')
    arm_box_angles    = LaunchConfiguration('arm_box_angles')

    shuttle = PathJoinSubstitution([
        FindPackageShare('mirte_driving'), 'launch', 'shuttle.launch.py'])

    return LaunchDescription([
        # false = detection OFFLOADED to the laptop: run
        #         `ros2 launch mirte_driving detector.launch.py` there and the
        #         shuttle consumes its /zone_a_pose + /zone_b_pose over wifi.
        #         Frees the SBC (cv2 + camera stream off the robot).
        # true  = detect A/B on the robot (no laptop needed).
        DeclareLaunchArgument('run_zone_detector', default_value='false'),
        # NOTE: we publish NO odom→base_link and NO sensor-mount transforms.
        # The robot's own bringup already provides both — the base controller
        # (enable_odom_tf:true) broadcasts odom→base_link, and
        # robot_state_publisher broadcasts the base_link→laser / camera mounts
        # from the URDF.  Publishing them ourselves created two authorities for
        # one transform (TF_OLD_DATA spam, robot drove into a wall), so we
        # deleted our copies and rely on the robot.
        # Full place cycle at B (precise dock → lay-down → walk-back → home),
        # then back to A.  Set false for just the precise adjust then back to A.
        DeclareLaunchArgument('dock_wait_for_box', default_value='true'),
        # Real lidar is back → use it. Set true ONLY on a unit with no lidar
        # (synthesizes /scan from the depth camera; would double-publish otherwise).
        DeclareLaunchArgument('use_depth_scan', default_value='false'),
        # Handle grasp at A.  ALL perception (YOLO) runs ON THE LAPTOP — start
        # `ros2 launch mirte_perception grasp.launch.py` there; the robot only
        # publishes its cameras and consumes /perception/object_markers +
        # /grasp_handle over the network.  On each A arrival: wait for a handle
        # detection, call /grasp_handle once, proceed to B when it returns
        # (success or not).  grasp_at_a:=false skips the grasp entirely.
        # OFF for now: navigation + arm mimic only.  grasp_at_a spawns nothing on
        # the robot (it waits on the laptop's mirte_perception topics) and
        # dock_at_b spawns mirte_placement's marker_navigator/box_placer at B —
        # re-enable each with grasp_at_a:=true / dock_at_b:=true when those
        # stacks are back in the loop.  The camera align at A/B still runs (it's
        # part of the navigation), and arm_mimic still does the grab/drop poses.
        DeclareLaunchArgument('grasp_at_a', default_value='false'),
        DeclareLaunchArgument('dock_at_b',   default_value='false'),
        DeclareLaunchArgument('arm_mimic',   default_value='true'),
        DeclareLaunchArgument('round_trips', default_value='3'),
        # UDP-only DDS for OUR nodes only (fixes the SHM open_and_lock errors on
        # units whose boot service owns /dev/shm).  Scoped to this launch — no
        # ~/.bashrc export, so teleop and other shells are untouched.  Set false
        # on a robot whose bringup runs the Fast-DDS discovery server.
        DeclareLaunchArgument('udp_only', default_value='true'),
        # 0.40 (not the package's 0.25 default): in the box-carry pose this
        # unit's gripper sits in the lidar plane ~0.3 m ahead — at 0.25 those
        # returns get MARKED inside the 0.32 m footprint + carry inflation and
        # Nav2 reports 'collision ahead' for EVERY motion (backup included) the
        # moment the arm curls at A, freezing the A→B leg.  Raise to 0.45 if
        # the lockup still appears right after the arm moves.
        DeclareLaunchArgument('scan_min_range', default_value='0.40'),
        # The ONLY two arm poses of the whole shuttle, [shoulder_pan,
        # shoulder_lift, elbow, wrist] rad, actively held (re-sent every 4 s so
        # the joints stay energized and the arm can't sag):
        #   up  = upright/all-zero, everywhere except the carry;
        #   box = the gripping pose, from the A pickup until the B drop.
        # Joint sign conventions DIFFER PER UNIT — if a pose folds the arm the
        # wrong way, find the right angles with a manual `ros2 topic pub
        # /mirte_master_arm_controller/joint_trajectory` and override here.
        DeclareLaunchArgument('arm_up_angles',  default_value='[0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('arm_box_angles', default_value='[0.0, -1.2, -1.5, 1.4]'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([shuttle]),
            launch_arguments={
                'use_sim_time':      'false',
                'provide_sim_tf':    'false',
                'run_zone_detector': run_zone_detector,
                'dock_wait_for_box': dock_wait_for_box,
                'use_depth_scan':    use_depth_scan,
                # The real camera's raw color stream is lazy/unreliable; the
                # compressed (JPEG) stream is always there and ~20x lighter on
                # the SBC — robot-side zone_detector must use it.
                'use_compressed':    'true',
                'aruco_dict':        'DICT_4X4_250',
                'zone_a_id':         '100',
                'zone_b_left_id':    '101',
                'zone_b_right_id':   '102',
                'zone_marker_size':  '0.08',
                'dock_at_b':         dock_at_b,
                'arm_mimic':         arm_mimic,
                'round_trips':       round_trips,
                'grasp_at_a':        grasp_at_a,
                'image_topic':       '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'cmd_vel_topic':     '/mirte_base_controller/cmd_vel',
                # The real base NAMESPACES its odometry; nothing publishes a
                # bare /odom on the robot.  Without this, Nav2's odom smoother
                # reads zero velocity and RPP's velocity-scaled lookahead pins
                # at its minimum (twitchy, corner-cutting path following).
                'odom_topic':        '/mirte_base_controller/odom',
                'scan_min_range':    scan_min_range,
                'arm_up_angles':     arm_up_angles,
                'arm_box_angles':    arm_box_angles,
                # Was declared above but never forwarded — the shuttle launch's
                # own default (false) silently won and the UDP-only profile
                # never applied on this unit.
                'udp_only':          udp_only,
            }.items()),
    ])
