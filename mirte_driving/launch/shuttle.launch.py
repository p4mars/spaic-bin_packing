"""
shuttle.launch.py — brings up the WHOLE driving stack (mapping + navigation).

This is the assembly file: it starts every node of mirte_driving plus the
external SLAM and Nav2 servers, in the right order, wired to the right topics.
mission.launch.py just calls this with the real-robot argument values baked in.

═══════════════════════════════════════════════════════════════════════════════
NODE GRAPH IT BRINGS UP  (and the data that flows between them)
═══════════════════════════════════════════════════════════════════════════════
  [lidar]→ scan_filter ─/scan_filtered─┬─► slam_toolbox ─/map, map→odom TF
                                       └─► Nav2 costmaps (obstacle layer)
  [camera]→ zone_detector ─/zone_a_pose,/zone_b_pose (only if run_zone_detector;
                                                       else the laptop does it)
  Nav2 (planner, controller, behaviors, bt_navigator, lifecycle_mgr, 2 costmaps)
        ──► follows paths, emits Twist on cmd_vel_topic
  shuttle_manager ──NavigateToPose──► Nav2 ;  ──Twist──► base (spins/aligns/turn)
  (pointcloud_to_laserscan only on a no-lidar unit: depth cloud → /scan)
  NOT here: odom→base_link and the sensor-mount transforms — the robot's own
  bringup already publishes those (see the note at the args), so we don't.

═══════════════════════════════════════════════════════════════════════════════
STARTUP TIMELINE  (TimerAction offsets — staged so each layer's prerequisites
exist before it starts, and the SBC's DDS bus isn't swamped all at once)
═══════════════════════════════════════════════════════════════════════════════
  t=0   scan_filter, zone_detector, (sim-only base_footprint static), DDS profile
  t=5   slam_toolbox  (SYNC node — processes every scan; the async node DROPPED
        scans under load and smeared rotated copies of the room into the map)
  t=35  Nav2 container — needs map→odom to exist first, and the delay lets the
        DDS discovery storm settle so lifecycle activation doesn't time out
  t=50  shuttle_manager — last, so SLAM + Nav2 are live before it sends goals

═══════════════════════════════════════════════════════════════════════════════
KEY DESIGN CHOICES (the "why"; expanded per-argument below)
═══════════════════════════════════════════════════════════════════════════════
  • Nav2 is COMPOSED into one component_container_isolated (one DDS participant):
    5 separate servers flooded multicast discovery on this SBC and lifecycle
    activation timed out.
  • SYNC slam_toolbox (not async): never drop scans during spins.
  • cmd_vel / odom topic names are ARGS: sim uses /cmd_vel + /odom (planar_move
    plugin); the real robot uses /mirte_base_controller/{cmd_vel,odom}.
  • Per-unit transport hazards are ARGS, not edits: udp_only (shared-memory DDS
    clash), scan_min_range (arm in the lidar plane).  We do NOT publish
    odom→base_link or the sensor-mount transforms — the robot's own bringup
    already does (base controller enable_odom_tf + robot_state_publisher).

SIM (default):   ros2 launch mirte_driving shuttle.launch.py
REAL ROBOT:      normally just use mission.launch.py (sets all of the below for
                 hardware).  Direct form, after the robot's own bringup is up
                 (camera, lidar /scan, base odom + odom→base_link TF, cmd_vel):
    ros2 launch mirte_driving shuttle.launch.py \
        use_sim_time:=false provide_sim_tf:=false \
        aruco_dict:=DICT_4X4_250 zone_a_id:=100 \
        zone_b_left_id:=101 zone_b_right_id:=102 \
        image_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info \
        cmd_vel_topic:=/mirte_base_controller/cmd_vel \
        odom_topic:=/mirte_base_controller/odom
  provide_sim_tf:=false is REQUIRED on hardware: it skips the sim-only
  base_footprint/base_frame statics.  (check real topic names with
  `ros2 topic list`).
"""

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    aruco_dict   = LaunchConfiguration('aruco_dict')
    zone_a_id        = LaunchConfiguration('zone_a_id')
    zone_b_left_id   = LaunchConfiguration('zone_b_left_id')
    zone_b_right_id  = LaunchConfiguration('zone_b_right_id')
    zone_marker_size = LaunchConfiguration('zone_marker_size')
    round_trips  = LaunchConfiguration('round_trips')
    approach_dist = LaunchConfiguration('approach_dist')
    approach_dist_a = LaunchConfiguration('approach_dist_a')
    align_at_a      = LaunchConfiguration('align_at_a')
    align_at_b      = LaunchConfiguration('align_at_b')
    dock_at_b    = LaunchConfiguration('dock_at_b')
    arm_mimic    = LaunchConfiguration('arm_mimic')
    arm_up_angles    = LaunchConfiguration('arm_up_angles')
    arm_box_angles   = LaunchConfiguration('arm_box_angles')
    dock_approach_dist = LaunchConfiguration('dock_approach_dist')
    dock_wait_for_box = LaunchConfiguration('dock_wait_for_box')
    dock_marker_size   = LaunchConfiguration('dock_marker_size')
    dock_image_topic   = LaunchConfiguration('dock_image_topic')
    dock_info_topic    = LaunchConfiguration('dock_info_topic')
    dock_cmd_vel_topic = LaunchConfiguration('dock_cmd_vel_topic')
    dock_approach_m    = LaunchConfiguration('dock_approach_m')
    dock_seek_dist     = LaunchConfiguration('dock_seek_dist')
    grasp_at_a           = LaunchConfiguration('grasp_at_a')
    grasp_detect_timeout = LaunchConfiguration('grasp_detect_timeout')
    grasp_timeout        = LaunchConfiguration('grasp_timeout')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic    = LaunchConfiguration('odom_topic')
    image_topic   = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    provide_sim_tf = LaunchConfiguration('provide_sim_tf')
    use_compressed = LaunchConfiguration('use_compressed')
    run_zone_detector = LaunchConfiguration('run_zone_detector')
    use_depth_scan = LaunchConfiguration('use_depth_scan')
    udp_only = LaunchConfiguration('udp_only')
    scan_min_range = LaunchConfiguration('scan_min_range')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Sim arena now matches the REAL marker scheme: DICT_4X4_250, Zone A
        # pole = id 100, Zone B = ids 101/102 glued on the east wall.  Only the
        # marker size differs (sim panels 0.15 m vs 0.08 m printed).
        DeclareLaunchArgument('aruco_dict',   default_value='DICT_4X4_250'),
        DeclareLaunchArgument('zone_a_id',       default_value='100'),
        # Zone B is the precision pair's TWO markers; /zone_b_pose = midpoint.
        DeclareLaunchArgument('zone_b_left_id',  default_value='101'),
        DeclareLaunchArgument('zone_b_right_id', default_value='102'),
        DeclareLaunchArgument('zone_marker_size', default_value='0.15'),     # real: 0.08 (printed size)
        DeclareLaunchArgument('round_trips',  default_value='3'),
        # Hand off the precise B docking to the precision team (marker_navigator +
        # box_placer): at B the shuttle stops, publishes /start_docking, and waits
        # for /robot_backed_up before the B→A leg.  Set false for the stand-alone
        # single-target shuttle with the arm-carry mimic.
        DeclareLaunchArgument('dock_at_b',          default_value='true'),
        # false = NAVIGATION-ONLY: never move the arm at A/B (only matters when
        # dock_at_b/grasp_at_a are off — those modes already own the arm).
        DeclareLaunchArgument('arm_mimic',          default_value='true'),
        # The ONLY two arm poses of the whole shuttle, [shoulder_pan,
        # shoulder_lift, elbow, wrist] rad — held actively (re-sent every few
        # seconds) so the joints can't sag:
        #   up:  upright/all-zero — startup, search, and everywhere except A→B.
        #   box: the gripping pose — from the A pickup until the B drop.
        # Joint SIGN CONVENTIONS DIFFER PER UNIT (the old box pose tucked the
        # arm BACKWARD on one robot).  Find the pose with a manual
        # `ros2 topic pub /mirte_master_arm_controller/joint_trajectory …` and
        # override here; no rebuild needed.
        DeclareLaunchArgument('arm_up_angles',  default_value='[0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('arm_box_angles', default_value='[0.0, -1.2, -1.5, 1.4]'),
        # Standoff (m) for the B leg when docking — stop further back so BOTH B
        # markers stay in the camera FOV for marker_navigator's precise dock.
        DeclareLaunchArgument('dock_approach_dist', default_value='0.5'),
        # true = run the FULL place cycle at B (spawn box_placer too; bridge
        # /robot_positioned→/start_placing; resume on /robot_backed_up — i.e. the
        # lay-down + walk-back).  false = just the precise adjust, then back to A.
        DeclareLaunchArgument('dock_wait_for_box', default_value='false'),
        # Settings handed to the SPAWNED marker_navigator/box_placer (the friend's
        # scripts, configured via params/remaps only).  Defaults = real robot;
        # the sim mission overrides camera/cmd_vel/sizes.
        DeclareLaunchArgument('dock_marker_size',   default_value='0.08'),
        DeclareLaunchArgument('dock_image_topic',   default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('dock_info_topic',    default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('dock_cmd_vel_topic', default_value='/mirte_base_controller/cmd_vel'),
        # < 0 → keep marker_navigator's own defaults (approach 0.40 / seek 0.22), whatever precision team defined.
        DeclareLaunchArgument('dock_approach_m',    default_value='-1.0'),
        DeclareLaunchArgument('dock_seek_dist',     default_value='-1.0'),
        # Handle grasp at A.  The mirte_perception stack (YOLO) runs ON THE
        # LAPTOP (`ros2 launch mirte_perception grasp.launch.py` there); the
        # shuttle only consumes /perception/object_markers + /grasp_handle over
        # the network: on reaching A it waits for a handle detection, calls the
        # service once, and proceeds to B when it returns (success or not).
        DeclareLaunchArgument('grasp_at_a',           default_value='false'),
        DeclareLaunchArgument('grasp_detect_timeout', default_value='30.0'),
        DeclareLaunchArgument('grasp_timeout',        default_value='180.0'),
        # Distance (m) from the marker to the robot CENTRE at the approach
        # standoff.  Front bumper is ~0.20 m ahead of base_link, so 0.1 m puts
        # the robot's front right up against the marker.  Override here instead of
        # editing the source (editing source on the robot blocks `git pull`).
        # 1.0 m (was 0.3): with the arm-mimic pose extended forward, a 0.3 m
        # B standoff put wall+arm inside the collision check — the B goal kept
        # aborting (so the B-align never even triggered) and the arm grazed the
        # wall.  B now keeps the same 1 m standoff as A.
        DeclareLaunchArgument('approach_dist', default_value='1.0'),
        # Zone A: Nav2 stops ~this far from tag A, then (align_at_a) a camera
        # P-servo centres the robot EXACTLY in front of the tag at this same
        # distance, facing it — like B's precise dock, but single-tag and the
        # robot keeps its standoff.
        DeclareLaunchArgument('approach_dist_a', default_value='1.0'),
        DeclareLaunchArgument('align_at_a',      default_value='true'),
        # B fine-alignment is part of the NAVIGATION code (runs with or without
        # the placement merge); dock_at_b only adds marker_navigator/box_placer.
        DeclareLaunchArgument('align_at_b',      default_value='true'),
        # SIM: the robot body is moved by the URDF's gazebo_planar_move plugin,
        # which listens on /cmd_vel (the ros2_control wheel chain accepts commands
        # but does not actuate the body in gazebo).  REAL robot: the mission
        # launch overrides this with /mirte_base_controller/cmd_vel.
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        # Where Nav2 reads the robot's odometry VELOCITY (controller_server's
        # and bt_navigator's odom smoother).  SIM: planar_move publishes /odom
        # directly.  REAL robot: the base namespaces it — the mission passes
        # /mirte_base_controller/odom.  Left at a silent /odom, RPP's
        # velocity-scaled lookahead reads zero speed and pins at
        # min_lookahead_dist forever (twitchy, corner-cutting following).
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('image_topic',       default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        # SIM provides odom relay + base_footprint/base_frame static TF.  The
        # REAL robot's own bringup already publishes these (and would conflict),
        # so set provide_sim_tf:=false on hardware.
        DeclareLaunchArgument('provide_sim_tf',    default_value='true'),
        # Real robot: subscribe to the camera's compressed (JPEG) stream instead
        # of raw — ~20x less data to deserialize on the SBC.  Sim publishes raw.
        DeclareLaunchArgument('use_compressed',    default_value='false'),
        # Set false to OFFLOAD zone_detector to the laptop (run detector.launch.py
        # there).  Frees the SBC's DDS bus for SLAM's /tf so localization stops
        # drifting under full mission load.
        DeclareLaunchArgument('run_zone_detector', default_value='true'),
        # NOTE on transforms we do NOT publish: the robot's own bringup already
        # broadcasts odom→base_link (its base controller has enable_odom_tf:true)
        # and the base_link→laser / base_link→camera_link sensor mounts (its
        # robot_state_publisher, from the URDF).  We deliberately do NOT duplicate
        # them — two publishers of the same transform fight and corrupt
        # localization.  On a unit whose bringup genuinely lacks these, fix it in
        # the bringup, not here.
        # On a unit with NO lidar, synthesize /scan from the depth camera
        # (pointcloud_to_laserscan off /camera/depth/points).  Needs
        # ros-humble-pointcloud-to-laserscan.  Forward cone only — weaker than a
        # 360° lidar, but a real scan SLAM/costmaps can use.
        DeclareLaunchArgument('use_depth_scan', default_value='false'),
        # Force UDP-only DDS for THE NODES OF THIS LAUNCH ONLY (fixes the
        # "RTPS_TRANSPORT_SHM ... open_and_lock_file failed" SHM clash with the
        # boot service on some units).  Scoped here on purpose: do NOT export
        # FASTRTPS_DEFAULT_PROFILES_FILE in ~/.bashrc — that hits every shell
        # (teleop included) and breaks discovery-server (MIRTE_FASTDDS=true)
        # setups.  Leave false on robots whose bringup uses the discovery server.
        DeclareLaunchArgument('udp_only', default_value='false'),
        # Lidar self-return cutoff.  RAISE (e.g. 0.40) on a unit whose arm or
        # gripper sits in the lidar plane ~0.25-0.35 m ahead: those returns land
        # INSIDE the 0.32 m footprint and Nav2 then reports 'collision ahead'
        # for every motion, backup included, without moving at all.
        DeclareLaunchArgument('scan_min_range', default_value='0.25'),
    ]

    nav_params = PathJoinSubstitution([
        FindPackageShare('mirte_driving'), 'params', 'exploration_nav2_params.yaml'])
    slam_params = PathJoinSubstitution([
        FindPackageShare('mirte_driving'), 'params', 'slam_params.yaml'])
    bt_xml = PathJoinSubstitution([
        FindPackageShare('mirte_driving'), 'trees', 'nav2_minimal_tree.xml'])

    sim = {'use_sim_time': use_sim_time}

    return LaunchDescription(args + [

        # udp_only:=true → UDP-only Fast-DDS profile for the processes of THIS
        # launch only (SetEnvironmentVariable scopes to launched children, not
        # the shell), sidestepping root/service-owned /dev/shm segments.  Other
        # terminals (teleop!) and discovery-server setups are unaffected.
        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE',
            value=PathJoinSubstitution([FindPackageShare('mirte_driving'),
                                        'config', 'fastdds_udp_only.xml']),
            condition=IfCondition(udp_only)),

        # No-lidar units: build /scan from the depth camera's point cloud.  Output
        # in base_link as a horizontal slice; scan_filter then makes /scan_filtered.
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan', output='screen',
             condition=IfCondition(use_depth_scan),
             remappings=[('cloud_in', '/camera/depth/points'), ('scan', '/scan')],
             parameters=[sim, {'target_frame': 'base_link',
                               'transform_tolerance': 0.1,
                               'min_height': 0.08, 'max_height': 0.50,
                               'angle_min': -1.0, 'angle_max': 1.0,
                               'angle_increment': 0.0087, 'scan_time': 0.1,
                               'range_min': 0.2, 'range_max': 5.0, 'use_inf': True}]),

        Node(package='mirte_driving', executable='scan_filter.py',
             name='scan_filter', output='screen',
             parameters=[sim, {'min_range': scan_min_range}]),

        # Zone detector — marker IDs/dict are params so the same node works in
        # sim (A=0, B=1/2, 4x4_50) and on the robot (A=104, B=101/102, 4x4_250).
        Node(package='mirte_driving', executable='zone_detector.py',
             name='zone_detector', output='screen',
             condition=IfCondition(run_zone_detector),   # false → run it on the laptop
             parameters=[sim, {'aruco_dict': aruco_dict,
                               'zone_a_id': zone_a_id,
                               'zone_b_left_id': zone_b_left_id,
                               'zone_b_right_id': zone_b_right_id,
                               'zone_marker_size': zone_marker_size,
                               'use_compressed': use_compressed}],
             remappings=[('/camera/image_raw', image_topic),
                         ('/camera/image_raw/compressed',
                          [image_topic, TextSubstitution(text='/compressed')]),
                         ('/camera/camera_info', camera_info_topic)]),

      #If provide_sim_tf:=true
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
             output='screen', parameters=[sim],
             condition=IfCondition(provide_sim_tf)),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
             output='screen', parameters=[sim],
             condition=IfCondition(provide_sim_tf)),

        # NOTE: we publish NO odom→base_link and NO sensor-mount transforms.
        # On the REAL robot the bringup already provides both (base controller
        # enable_odom_tf:true → odom→base_link; robot_state_publisher → the
        # base_link→laser/camera mounts from the URDF).  In SIM the gazebo
        # plugins provide them.  Duplicating either would create two publishers
        # of one transform and corrupt localization, so we leave them to the
        # robot/sim.  (The base_footprint/base_frame statics above are sim-only
        # helper frames gazebo doesn't make, hence provide_sim_tf, false on hw.)

        TimerAction(period=5.0, actions=[
            # SYNC node, like the proven minimal_slam stack: processes every
            # scan in order.  The async node "gracefully" DROPS scans under
            # load — and dropped scans during spins are exactly what lost the
            # matcher's lock and stamped rotated copies of the room into the
            # map.  The CPU headroom exists now (detector offloaded, Nav2
            # composed).
            Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
                 name='slam_toolbox', output='screen', parameters=[slam_params, sim]),
        ]),

        # COMPOSED Nav2: all five servers run in ONE container, sharing a single
        # DDS participant.  On this robot MIRTE_FASTDDS=false, so ~24 separate
        # participants (mirte-ros + our stack) flood multicast discovery — it took
        # 25 s just to discover planner_server/get_state, and the lifecycle
        # manager's service calls then timed out ("async_send_request failed"),
        # aborting bringup.  In one container the lifecycle get_state/change_state
        # calls are intra-process (no DDS discovery), so they can't time out, and
        # Nav2 contributes 1 participant instead of 5.
        TimerAction(period=35.0, actions=[
            ComposableNodeContainer(
                name='nav2_container', namespace='',
                package='rclcpp_components', executable='component_container_isolated',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_planner', plugin='nav2_planner::PlannerServer',
                        name='planner_server', parameters=[nav_params, sim]),
                    ComposableNode(
                        package='nav2_controller', plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[nav_params, sim, {'odom_topic': odom_topic}],
                        remappings=[('cmd_vel', cmd_vel_topic)]),
                    ComposableNode(
                        package='nav2_behaviors', plugin='behavior_server::BehaviorServer',
                        name='behavior_server', parameters=[nav_params, sim],
                        remappings=[('cmd_vel', cmd_vel_topic)]),
                    ComposableNode(
                        package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator', parameters=[nav_params, sim,
                            {'default_nav_to_pose_bt_xml': bt_xml,
                             'default_nav_through_poses_bt_xml': bt_xml,
                             'odom_topic': odom_topic}]),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                                     'bond_timeout': 0.0,
                                     'node_names': ['planner_server', 'controller_server',
                                                    'behavior_server', 'bt_navigator']}]),
                ]),
        ]),

        TimerAction(period=50.0, actions=[
            Node(package='mirte_driving', executable='shuttle_manager.py',
                 name='shuttle_manager', output='screen',
                 parameters=[sim, {'round_trips': round_trips,
                                   'approach_dist': approach_dist,
                                   'approach_dist_a': approach_dist_a,
                                   'align_at_a': align_at_a,
                                   'align_at_b': align_at_b,
                                   'dock_at_b': dock_at_b,
                                   'arm_mimic': arm_mimic,
                                   # ParameterValue(value_type=None) parses the
                                   # arg string as YAML → double array.
                                   'arm_up_angles': ParameterValue(
                                       arm_up_angles, value_type=None),
                                   'arm_box_angles': ParameterValue(
                                       arm_box_angles, value_type=None),
                                   'dock_approach_dist': dock_approach_dist,
                                   'dock_wait_for_box': dock_wait_for_box,
                                   'dock_marker_size': dock_marker_size,
                                   'dock_image_topic': dock_image_topic,
                                   'dock_info_topic': dock_info_topic,
                                   'dock_cmd_vel_topic': dock_cmd_vel_topic,
                                   'dock_approach_m': dock_approach_m,
                                   'dock_seek_dist': dock_seek_dist,
                                   'grasp_at_a': grasp_at_a,
                                   'grasp_detect_timeout': grasp_detect_timeout,
                                   'grasp_timeout': grasp_timeout,
                                   'cmd_vel_topic': cmd_vel_topic}]),
        ]),
    ])
