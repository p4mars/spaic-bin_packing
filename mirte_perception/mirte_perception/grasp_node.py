import math
import os
import time
import warnings
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient  # for GripperCommand
from rclpy.callback_groups import ReentrantCallbackGroup  # to handle simultaneous callbacks
from rclpy.executors import MultiThreadedExecutor  # thread pool for ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from ament_index_python.packages import get_package_share_directory  # to find URDF file

import tf2_ros  # coordinate transform library
import tf2_geometry_msgs

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    from ikpy.chain import Chain


_GRIPPER_OPEN = -0.3
_GRIPPER_CLOSE = 0.3502
_GRASP_DETECT_THRESHOLD = 0.08  # rad: gripper must stop this far below _GRIPPER_CLOSE to confirm a handle
_GRASP_MAX_RETRIES      = 2     # re-attempts after a slip or servo timeout before giving up
_GRASP_BACKUP_M         = 0.10  # metres to back up before retrying
_GRASP_Z_M = 0.06        # height at which the gripper closes (metres above base_link origin)
_APPROACH_Z_M = 0.15     # pick-up height
_CALIB_X_M = 0.03        # offsets for depth camera
_CALIB_Y_M = 0.02
_MOVE_DURATION_SEC = 7   # time for arm to complete a move
_ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']

# Top-down YOLO base-servo parameters
_HOVER_Z_M       = 0.20   # arm hover height above floor 
_HOVER_WRIST_RAD = -math.pi / 2  # wrist angle for camera-down pose 
_SERVO_KP        = 1.0    # proportional gain: metric error → base velocity (m/s per m)
_SERVO_FAST_VEL  = 0.06   # velocity used when handle is far from target (coarse phase)
_SERVO_MAX_VEL   = 0.035  # max base linear velocity m/s (fine approach, near target)
_SERVO_MIN_VEL   = 0.035   # min velocity that actually moves the robot (motor dead zone)
_SERVO_BACKUP_VEL = 0.07  # backup velocity 
_SERVO_CENTER_PX = 20     # convergence threshold in pixels
_SERVO_BASE_ITERS = 300   # max servo iterations 
_SERVO_CAM_DX_PX = 0      # gripper camera optical centre → gripper jaw X pixel offset 
_SERVO_TARGET_Y_FRAC = 0.85  # vertical target as fraction of image height (bottom of frame)
_SERVO_SIGN_X    = -1     # flip sign if base moves wrong direction in X 
_SERVO_SIGN_Y    = -1     # flip sign if base moves wrong direction in Y 
_SERVO_FOCAL_DEFAULT = 400.0  # fallback focal length for uncalibrated gripper camera
_SERVO_FORWARD_OFFSET_M = 0.055  # extra forward nudge after servo convergence (metres)
_MAX_DEPTH_FALLBACKS        = 7    # abort visual servo after this many depth-camera nudges
_CENTROID_STALE_FRAMES      = 3    # frames to reuse last gripper centroid before declaring hard loss
_DEPTH_GRASP_APPROACH_DIST_M = 0.25  # approach until handle is within this distance before IK
_SERVO_COARSE_M = 0.10             # metric error above which coarse open-loop drive is used
_SERVO_FRAME_TIMEOUT = 0.15        # max seconds to wait for a genuinely new camera frame
_SERVO_STEP_SEC = 0.05             # fine-phase step duration (loop-published to keep controller alive)

# Fixed hover joint angles [shoulder_pan, shoulder_lift, elbow, wrist] (radians)
# Found empirically — camera faces down at this pose
_HOVER_JOINTS = [0.0, -0.4329114676646735, -0.8915839950887833, -1.5707963267948966]

# frame_base_joint: xyz=[0,0,0.10], yaw=+90° — transforms base_link → frame_link
_FRAME_Z_OFFSET = 0.10

# `base_link`: X=forward, Y=left, Z=up, origin at floor level
# `frame_link`: the shoulder of the arm. The URDF defines it as translated 10 cm up from `base_link` and rotated +90° around Z.

def _base_to_frame(pos):
    """Convert a point from base_link to frame_link coordinates."""
    bx, by, bz = pos
    return np.array([by, -bx, bz - _FRAME_Z_OFFSET])


def _frame_to_base(pos):
    """Convert a point from frame_link to base_link coordinates."""
    fx, fy, fz = pos
    return np.array([-fy, fx, fz + _FRAME_Z_OFFSET])


class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')

        self._cb = ReentrantCallbackGroup() # all subscriptions, action client and service assigned to self._cb

        self.tf_buffer = tf2_ros.Buffer() # answers lookup queries
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) # feeds the buffer

        # handle state
        self._latest_handles = [] # all handle markers from most recent perception message
        self._handle_history = deque(maxlen=5) # 5 most recent best handles
        self._last_handle_time = None # when last handle was seen
        self._last_gripper_centroid = None   # (cx, cy) from last successful YOLO detection
        self._servo_exit_reason = ''         # reason for last _visual_servo_base() failure
        
        # receive 3D handle positions from the perception node continuously
        self.create_subscription(
            MarkerArray, '/perception/object_markers',
            self._markers_cb, 10, callback_group=self._cb,
        )

        self._traj_pub = self.create_publisher(
            JointTrajectory, '/mirte_master_arm_controller/joint_trajectory', 10,
        )

        self._gripper_client = ActionClient(
            self, GripperCommand, '/mirte_master_gripper_controller/gripper_cmd',
            callback_group=self._cb,
        )

        self.create_service(Trigger, '/grasp_handle', self._grasp_cb, callback_group=self._cb)

        # Base velocity publisher for visual servo
        self._cmd_vel_pub = self.create_publisher(Twist, '/mirte_base_controller/cmd_vel', 10)

        self._grasp_proceed_pub = self.create_publisher(Bool, '/grasp/proceed', 10)

        # Gripper camera subscriptions
        self._latest_gripper_img = None
        self._gripper_cam_info = None
        self._bridge = None

        # store latest messages
        self.create_subscription(
            Image, '/gripper_camera/image_raw',
            lambda msg: setattr(self, '_latest_gripper_img', msg),
            10, callback_group=self._cb,
        )
        self.create_subscription(
            CameraInfo, '/gripper_camera/camera_info',
            lambda msg: setattr(self, '_gripper_cam_info', msg),
            10, callback_group=self._cb,
        )
        self._servo_annotated_pub = self.create_publisher(
            Image, '/gripper_camera/annotated_image', 10,
        )

        # YOLO model for top-down handle detection via gripper camera
        self._gripper_yolo = None
        gripper_model_path = self.declare_parameter('gripper_model_path', '').value
        if gripper_model_path:
            try:
                from ultralytics import YOLO
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore')
                    self._gripper_yolo = YOLO(os.path.expanduser(gripper_model_path))
                self.get_logger().info(f'Gripper YOLO model loaded: {gripper_model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load gripper YOLO model: {e}')
        else:
            self.get_logger().warn(
                'gripper_model_path not set — top-down YOLO detection disabled')

        # Build ikpy kinematic chain from URDF (frame_link → wrist)
        try:
            urdf_path = self._find_urdf()
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                self._chain = Chain.from_urdf_file(
                    urdf_path,
                    base_elements=['frame_link', 'shoulder_pan_joint'],
                    last_link_vector=np.zeros(3),
                    base_element_type='link',
                )
            self._chain.active_links_mask = [False, True, True, True, True, False, False] # IK skip base frame, solve 4 arm joints, skip end-effector links
            self.get_logger().info('IK chain loaded — 4 DOF arm ready.')
        except Exception as e:
            self.get_logger().error(f'Failed to load IK chain: {e}')
            self._chain = None

        self.get_logger().info('GraspNode ready — call /grasp_handle to trigger a grasp.')

    # ------------------------------------------------------------------
    # 1. Finds the xacro file in the installed package directory
    # 2. Runs `xacro` to expand macros and produce a plain URDF string
    # 3. Writes it to a temporary file
    # 4. Returns the temp file path for ikpy to read
    def _find_urdf(self):
        import subprocess, tempfile
        xacro_path = os.path.join(
            get_package_share_directory('mirte_master_description'),
            'urdf', 'mirte_master.xacro',
        )
        result = subprocess.run(
            ['ros2', 'run', 'xacro', 'xacro', xacro_path],
            capture_output=True, text=True, check=True,
        )
        tmp = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, mode='w')
        tmp.write(result.stdout)
        tmp.close()
        return tmp.name

    # ------------------------------------------------------------------
    # Filters markers to only receive from model1. Picks closest handle. Holds 5 most recent readings
    def _markers_cb(self, msg):
        handles = [m for m in msg.markers if m.ns == 'model1_sphere']
        self._latest_handles = handles
        if handles:
            best = min(handles, key=lambda m: m.pose.position.z)
            self._handle_history.append(best)
            self._last_handle_time = time.time()

    # ------------------------------------------------------------------
    def _compute_ik(self, pos_base_link, preferred_seed=None, fixed_wrist=None,
                    target_orientation=None):
        """Compute arm joint angles for a position in base_link.

        preferred_seed: tried first; generic seeds follow so the best is always picked.
        fixed_wrist: wrist joint held inactive at this value.
        target_orientation: 3x3 rotation matrix (in frame_link) to constrain end-effector
            orientation. When provided, ikpy uses orientation_mode='all'.
        """
        if self._chain is None:
            return None

        pos_frame = _base_to_frame(pos_base_link)

        target = np.eye(4)
        target[:3, 3] = pos_frame # desired position in the translation column
        orientation_mode = None
        # if orientation is provided, put it into the rotation block
        if target_orientation is not None:
            target[:3, :3] = target_orientation
            orientation_mode = 'all'

        original_mask = None
        # if wrist fixed, remove wrist joint from the chain
        if fixed_wrist is not None:
            original_mask = list(self._chain.active_links_mask)
            self._chain.active_links_mask = [False, True, True, True, False, False, False]

        try:
            wrist_val = fixed_wrist if fixed_wrist is not None else -math.pi / 2

            _seeds = [
                [0.0, 0.0,  0.00, -1.50, wrist_val, 0.0, 0.0],
                [0.0, 0.0, -0.30, -1.20, wrist_val, 0.0, 0.0],
                [0.0, 0.0,  0.30, -1.50, wrist_val, 0.0, 0.0],
                [0.0, 0.0, -0.50, -1.00, wrist_val, 0.0, 0.0],
            ]
            if preferred_seed is not None:
                seed = [0.0] + list(preferred_seed) + [0.0, 0.0]
                seed[4] = wrist_val
                _seeds = [seed] + _seeds

            best_result, best_error = None, float('inf')
            with warnings.catch_warnings():
                warnings.simplefilter('ignore')
                for seed in _seeds:
                    r = self._chain.inverse_kinematics_frame(
                        target, initial_position=seed, orientation_mode=orientation_mode)
                    fk = self._chain.forward_kinematics(r)
                    # run FK on IK result to confirm it actually reaches target
                    e = float(np.linalg.norm(fk[:3, 3] - pos_frame))
                    if e < best_error:
                        best_error = e
                        best_result = r

        finally:
            if original_mask is not None:
                self._chain.active_links_mask = original_mask

        joints = list(best_result[1:5])

        if fixed_wrist is not None:
            joints[3] = fixed_wrist

        if any(math.isnan(v) or math.isinf(v) for v in joints):
            self.get_logger().error('IK returned NaN/Inf — target may be out of reach')
            return None

        if best_error > 0.08:
            self.get_logger().error(
                f'IK FK error {best_error:.3f} m too large — target unreachable')
            return None

        self.get_logger().info(
            f'IK joints: {[f"{v:.3f}" for v in joints]}  FK error: {best_error:.4f} m'
        )
        return joints

    # ------------------------------------------------------------------
    # send single point trajectory to arm controller
    def _move_arm(self, joint_positions):
        traj = JointTrajectory()
        traj.joint_names = _ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = _MOVE_DURATION_SEC
        traj.points.append(point)
        self._traj_pub.publish(traj)
        time.sleep(_MOVE_DURATION_SEC + 0.5)

    # ------------------------------------------------------------------
    # run YOLO on the gripper camera and return pixel centroid of best detection
    def _detect_handle_top_down(self, img):
        """Run YOLO on a top-down gripper-camera frame. Returns (cx, cy) or None."""
        results = self._gripper_yolo(img, conf=0.5, verbose=False)
        best, best_conf = None, 0.0
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    best = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        return best

    # ------------------------------------------------------------------
    # drive the robot base to allign the handle under the gripper
    def _visual_servo_base(self):
        """Move the robot base to centre the handle under the gripper using top-down YOLO."""
        import cv2
        from cv_bridge import CvBridge
        if self._bridge is None:
            self._bridge = CvBridge()

        if self._gripper_yolo is None:
            self.get_logger().error('Base servo: no YOLO model loaded — skipping')
            return False


        # setup variables
        h, w = None, None
        no_detect_streak = 0 # consecutive frames where YOLO found nothing
        confirm_attempts = 0 # number of times the servo thought it converged but it didn't
        best_error_px = float('inf') # best pixel error seen so far for stuck detection
        iters_since_improvement = 0 # frames since meaningful error improvement (max 20)
        total_depth_fallbacks = 0 # number of depth fallbacks (max 7)
        coarse_moves = 0 # number of coarse moves (max 3)
        last_stamp = None
        _DEPTH_FALLBACK_AFTER = 5
        _STUCK_ITERS = 20
        _STUCK_IMPROVE_PX = 5.0
        _MAX_CONFIRM_ATTEMPTS = 3
        _LATERAL_BOUND_FRAC = 0.35

        for i in range(_SERVO_BASE_ITERS):
            # Wait for a genuinely new camera frame before acting
            deadline = time.time() + _SERVO_FRAME_TIMEOUT
            img_msg = None
            while time.time() < deadline:
                candidate = self._latest_gripper_img
                if candidate is not None and candidate.header.stamp != last_stamp:
                    img_msg = candidate
                    break
                time.sleep(0.02)
            if img_msg is None:
                img_msg = self._latest_gripper_img  # fall back to latest available (stale)
            if img_msg is None:
                no_detect_streak += 1
                self.get_logger().warn(f'Base servo iter {i}: no camera frame available')
                time.sleep(0.1)
                continue

            last_stamp = img_msg.header.stamp
            img = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            if h is None:
                h, w = img.shape[:2]

            detection = self._detect_handle_top_down(img)
            annotated = img.copy()

            # detection and stale centroid logic
            # if YOLO detects handle, use it, reset streak, update centroid
            # if YOLO misses but fewer than 3 frames missed, reuse last know centroid
            # if YOLO misses 5+ consecutive frames, depth camera fallback
            if detection is not None:
                self._last_gripper_centroid = detection
                no_detect_streak = 0
            elif self._last_gripper_centroid is not None and no_detect_streak < _CENTROID_STALE_FRAMES:
                # Smooth over transient missed frames by reusing the last known centroid
                no_detect_streak += 1
                detection = self._last_gripper_centroid
                self.get_logger().warn(
                    f'Base servo iter {i}: no detection, using stale centroid '
                    f'(miss={no_detect_streak})')
            else:
                # Sustained detection loss
                self._cmd_vel_pub.publish(Twist())
                no_detect_streak += 1
                if no_detect_streak >= _DEPTH_FALLBACK_AFTER:
                    self.get_logger().warn(
                        f'Base servo iter {i}: no detection for {no_detect_streak} frames '
                        f'— trying depth camera fallback')
                    self._depth_camera_nudge()
                    total_depth_fallbacks += 1
                    if total_depth_fallbacks >= _MAX_DEPTH_FALLBACKS:
                        self._cmd_vel_pub.publish(Twist())
                        self.get_logger().warn(
                            'Too many depth fallbacks — handing off to depth-camera grasp')
                        self._servo_exit_reason = 'depth_available'
                        self._last_gripper_centroid = None
                        return False
                    no_detect_streak = 0
                    time.sleep(0.5)
                else:
                    self.get_logger().warn(
                        f'Base servo iter {i}: no detection ({no_detect_streak}), base stopped')
                cv2.putText(annotated, f'NO DETECT streak={no_detect_streak}', (6, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)
                self._servo_annotated_pub.publish(
                    self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
                continue

            # error and target point
            cx, cy = detection
            target_x = w / 2.0 + _SERVO_CAM_DX_PX
            target_y = h * _SERVO_TARGET_Y_FRAC
            dx_px = cx - target_x
            dy_px = cy - target_y

            # Lateral bound guard: handle too far left/right — depth camera fallback
            if abs(dx_px) > w * _LATERAL_BOUND_FRAC:
                self.get_logger().warn(
                    f'Handle too far lateral (dx={dx_px:.0f}px, limit={w*_LATERAL_BOUND_FRAC:.0f}px)'
                    f' — depth camera fallback')
                self._cmd_vel_pub.publish(Twist())
                self._depth_camera_nudge()
                total_depth_fallbacks += 1
                if total_depth_fallbacks >= _MAX_DEPTH_FALLBACKS:
                    self._cmd_vel_pub.publish(Twist())
                    self.get_logger().warn(
                        'Too many depth fallbacks — handing off to depth-camera grasp')
                    self._servo_exit_reason = 'depth_available'
                    self._last_gripper_centroid = None
                    return False
                iters_since_improvement = 0
                best_error_px = float('inf')
                time.sleep(0.5)
                continue

            # Stuck detection: error not improving — depth camera fallback
            total_error_px = math.sqrt(dx_px ** 2 + dy_px ** 2)
            if best_error_px - total_error_px > _STUCK_IMPROVE_PX:
                best_error_px = total_error_px
                iters_since_improvement = 0
            else:
                iters_since_improvement += 1
            if iters_since_improvement >= _STUCK_ITERS:
                self.get_logger().warn(
                    f'Servo stuck for {_STUCK_ITERS} iters (err={total_error_px:.1f}px)'
                    f' — depth camera fallback')
                self._cmd_vel_pub.publish(Twist())
                self._depth_camera_nudge()
                total_depth_fallbacks += 1
                if total_depth_fallbacks >= _MAX_DEPTH_FALLBACKS:
                    self._cmd_vel_pub.publish(Twist())
                    self.get_logger().warn(
                        'Too many depth fallbacks — handing off to depth-camera grasp')
                    self._servo_exit_reason = 'depth_available'
                    self._last_gripper_centroid = None
                    return False
                iters_since_improvement = 0
                best_error_px = float('inf')
                time.sleep(0.5)
                continue

            # Safety guard: handle below danger line means robot is too close — back up
            if cy > h * 0.95:
                self.get_logger().warn(
                    f'Handle below danger line (cy={cy:.0f} > {h*0.95:.0f}) — backing up 2 cm')
                self._cmd_vel_pub.publish(Twist())
                backup = Twist()
                backup.linear.x = -_SERVO_BACKUP_VEL
                t0 = time.time()
                while time.time() - t0 < 0.05 / _SERVO_BACKUP_VEL:
                    self._cmd_vel_pub.publish(backup)
                    time.sleep(0.05)
                self._cmd_vel_pub.publish(Twist())
                time.sleep(0.5)
                iters_since_improvement = 0
                best_error_px = float('inf')
                continue

            # annotations
            cv2.circle(annotated, (int(cx), int(cy)), 8, (0, 255, 0), 2)
            cv2.drawMarker(annotated, (int(target_x), int(target_y)),
                           (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(annotated, f'dx={dx_px:.0f} dy={dy_px:.0f} iter={i}', (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
            self._servo_annotated_pub.publish(
                self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

            self.get_logger().info(
                f'Base servo iter {i}: dx={dx_px:.1f}px dy={dy_px:.1f}px')

            
            # convergence to target: stop, wait 2s for robot to settel, re-run YOLO on fresh frame, proceed or give up after 3 failed confirmations
            if abs(dx_px) < _SERVO_CENTER_PX and abs(dy_px) < _SERVO_CENTER_PX:
                self.get_logger().info(f'Base servo converged in {i} iterations — waiting 2s to confirm')
                self._cmd_vel_pub.publish(Twist())
                time.sleep(2.0)

                # Re-confirm with a fresh frame
                self._latest_gripper_img = None
                deadline = time.time() + 2.0
                while self._latest_gripper_img is None and time.time() < deadline:
                    time.sleep(0.05)
                if self._latest_gripper_img is None:
                    self.get_logger().warn('Confirmation: no fresh frame — proceeding anyway')
                    self._last_gripper_centroid = None
                    return True
                confirm_img = self._bridge.imgmsg_to_cv2(self._latest_gripper_img, 'bgr8')
                confirm = self._detect_handle_top_down(confirm_img)
                if confirm is None:
                    self.get_logger().warn('Confirmation: no detection after wait — continuing servo')
                    continue
                cdx = confirm[0] - target_x
                cdy = confirm[1] - target_y
                if abs(cdx) < _SERVO_CENTER_PX and abs(cdy) < _SERVO_CENTER_PX:
                    self.get_logger().info(f'Confirmation passed (cdx={cdx:.0f} cdy={cdy:.0f}) — proceeding')
                    self._last_gripper_centroid = None
                    return True
                confirm_attempts += 1
                self.get_logger().warn(
                    f'Confirmation failed ({confirm_attempts}/{_MAX_CONFIRM_ATTEMPTS})'
                    f' (cdx={cdx:.0f} cdy={cdy:.0f}) — resuming servo')
                if confirm_attempts >= _MAX_CONFIRM_ATTEMPTS:
                    self.get_logger().error('Max confirmation attempts reached — aborting servo')
                    self._cmd_vel_pub.publish(Twist())
                    self._servo_exit_reason = 'confirm_failed'
                    self._last_gripper_centroid = None
                    return False
                continue

            # pixel to meters conversion
            K = self._gripper_cam_info.k if self._gripper_cam_info is not None else None
            fx = float(K[0]) if (K is not None and float(K[0]) > 0) else _SERVO_FOCAL_DEFAULT
            fy = float(K[4]) if (K is not None and float(K[4]) > 0) else _SERVO_FOCAL_DEFAULT

            # pinhole camera formula
            err_x_m = dx_px * _HOVER_Z_M / fx
            err_y_m = dy_px * _HOVER_Z_M / fy
            total_err_m = math.sqrt(err_x_m**2 + err_y_m**2)

            # coarse phase: drive fast in the correct direction for time = distance / speed
            if total_err_m > _SERVO_COARSE_M and coarse_moves < 3:
                # Coarse phase: drive open-loop at FAST_VEL for the estimated time.
                # Loop-publish so the base controller doesn't time out mid-drive.
                drive_t = min(total_err_m / _SERVO_FAST_VEL, 0.3)
                coarse_moves += 1
                self.get_logger().info(
                    f'Base servo iter {i}: COARSE ({coarse_moves}/3) err={total_err_m:.3f}m driving {drive_t:.2f}s')
                twist = Twist()
                # x and y are switched in the camera compared to the base
                twist.linear.x = _SERVO_SIGN_X * (err_y_m / total_err_m) * _SERVO_FAST_VEL
                twist.linear.y = _SERVO_SIGN_Y * (err_x_m / total_err_m) * _SERVO_FAST_VEL
                t0 = time.time()
                # loop publish due to base controller watchdog
                while time.time() - t0 < drive_t:
                    self._cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                self._cmd_vel_pub.publish(Twist())
            else:
                # Fine phase: loop-publish for _SERVO_STEP_SEC so the controller
                # reliably executes the command, then stop and wait for the next frame.
                self.get_logger().info(
                    f'Base servo iter {i}: FINE err={total_err_m:.3f}m')

                def _clamp_vel(v):
                    v = float(np.clip(v, -_SERVO_MAX_VEL, _SERVO_MAX_VEL))
                    if 0 < abs(v) < _SERVO_MIN_VEL:
                        return math.copysign(_SERVO_MIN_VEL, v)
                    return v

                twist = Twist()
                twist.linear.x = _clamp_vel(_SERVO_KP * _SERVO_SIGN_X * err_y_m)
                twist.linear.y = _clamp_vel(_SERVO_KP * _SERVO_SIGN_Y * err_x_m)
                t0 = time.time()
                while time.time() - t0 < _SERVO_STEP_SEC:
                    self._cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                self._cmd_vel_pub.publish(Twist())

        self._cmd_vel_pub.publish(Twist())
        self.get_logger().warn('Base servo: max iterations reached without convergence')
        self._servo_exit_reason = 'timeout'
        self._last_gripper_centroid = None
        return False

    # ------------------------------------------------------------------
    # when gripper camera losed the handle, use _depth_camera_nudge
    # 1. Get the last depth-camera handle position (from `_handle_history`)
    # 2. Transform it to `base_link` using TF2
    # 3. Compute the current gripper position in `base_link` using FK
    # 4. The error is the difference: where the handle is minus where the gripper is
    # 5. Drive at half the error magnitude (gain = 0.5), clamped to `_SERVO_MAX_VEL`
    def _depth_camera_nudge(self):
        """Coarse base correction using depth camera when gripper camera loses the handle."""
        if not self._handle_history:
            self.get_logger().warn('Depth fallback: no handle history available')
            return
        if self._last_handle_time is not None and time.time() - self._last_handle_time > 2.0:
            self.get_logger().warn('Depth fallback: handle history too stale — skipping nudge')
            return
        best = self._handle_history[-1]
        handle_stamped = PoseStamped()
        handle_stamped.header = best.header
        handle_stamped.header.stamp = rclpy.time.Time().to_msg()
        handle_stamped.pose = best.pose
        try:
            handle_in_base = self.tf_buffer.transform(
                handle_stamped, 'base_link',
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'Depth fallback: TF failed: {e}')
            return

        # Gripper position in base_link from FK of current hover joints
        fk_mat = self._chain.forward_kinematics([0.0] + list(_HOVER_JOINTS) + [0.0, 0.0])
        gripper_base = _frame_to_base(fk_mat[:3, 3])

        hp = handle_in_base.pose.position
        err_x = hp.x - gripper_base[0]
        err_y = hp.y - gripper_base[1]
        self.get_logger().info(
            f'Depth fallback: handle=({hp.x:.2f},{hp.y:.2f}) '
            f'gripper=({gripper_base[0]:.2f},{gripper_base[1]:.2f}) '
            f'err=({err_x:.2f},{err_y:.2f})')

        if abs(err_x) < 0.01 and abs(err_y) < 0.01:
            return  # already close enough

        twist = Twist()
        twist.linear.x = float(np.clip(err_x * 0.5, -_SERVO_MAX_VEL, _SERVO_MAX_VEL))
        twist.linear.y = float(np.clip(err_y * 0.5, -_SERVO_MAX_VEL, _SERVO_MAX_VEL))
        self._cmd_vel_pub.publish(twist)
        time.sleep(0.3)
        self._cmd_vel_pub.publish(Twist())
        time.sleep(0.2)  # let image stabilise before next detection attempt

    # ------------------------------------------------------------------
    # used when the visual servo exits entirely
    def _depth_camera_grasp(self, hover_joints):
        """Grasp the handle using only the depth camera position when the gripper camera gave up."""
        if not self._handle_history:
            self.get_logger().error('Depth-camera grasp: no handle history')
            return False
        handle_age = time.time() - self._last_handle_time
        if handle_age > 3.0:
            self.get_logger().error(f'Depth-camera grasp: stale handle ({handle_age:.1f}s old)')
            return False

        fk_mat = self._chain.forward_kinematics([0.0] + list(hover_joints) + [0.0, 0.0])
        hover_orientation = fk_mat[:3, :3]

        # Approach phase: drive base toward handle until handle is within
        # _DEPTH_GRASP_APPROACH_DIST_M of the base_link origin
        for _ in range(200):
            if not self._handle_history:
                break
            handle_age = time.time() - self._last_handle_time
            if handle_age > 1.5:
                self.get_logger().warn(
                    f'Depth-camera approach: handle lost from depth camera for {handle_age:.1f}s '
                    f'— stopping approach (robot likely close enough)')
                self._cmd_vel_pub.publish(Twist())
                break
            best = self._handle_history[-1]
            handle_stamped = PoseStamped()
            handle_stamped.header = best.header
            handle_stamped.header.stamp = rclpy.time.Time().to_msg()
            handle_stamped.pose = best.pose
            try:
                handle_in_base = self.tf_buffer.transform(
                    handle_stamped, 'base_link',
                    timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warn(f'Depth-camera approach: TF failed: {e}')
                break
            hp = handle_in_base.pose.position
            depth_m = best.pose.position.z  # raw depth from the depth camera
            self.get_logger().info(
                f'Depth-camera approach: depth={depth_m:.3f}m handle=({hp.x:.2f},{hp.y:.2f})')
            if depth_m < _DEPTH_GRASP_APPROACH_DIST_M:
                self._cmd_vel_pub.publish(Twist())
                break
            horiz = math.sqrt(hp.x ** 2 + hp.y ** 2) or 1e-6
            twist = Twist()
            twist.linear.x = _SERVO_MAX_VEL * hp.x / horiz
            twist.linear.y = _SERVO_MAX_VEL * hp.y / horiz
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.3)
            self._cmd_vel_pub.publish(Twist())
            time.sleep(0.2)
        else:
            self._cmd_vel_pub.publish(Twist())
            self.get_logger().warn(
                'Depth-camera approach: did not converge in 50 steps — proceeding anyway')

        # Take a fresh averaged reading after approach for the IK target
        if not self._handle_history:
            self.get_logger().error('Depth-camera grasp: lost handle history after approach')
            return False
        handle_age = time.time() - self._last_handle_time
        if handle_age > 3.0:
            self.get_logger().error(f'Depth-camera grasp: stale after approach ({handle_age:.1f}s old)')
            return False
        xs = [m.pose.position.x for m in self._handle_history]
        ys = [m.pose.position.y for m in self._handle_history]
        best = self._handle_history[-1]
        handle_stamped = PoseStamped()
        handle_stamped.header = best.header
        handle_stamped.header.stamp = rclpy.time.Time().to_msg()
        handle_stamped.pose = best.pose
        handle_stamped.pose.position.x = sum(xs) / len(xs)
        handle_stamped.pose.position.y = sum(ys) / len(ys)
        try:
            handle_in_base = self.tf_buffer.transform(
                handle_stamped, 'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f'Depth-camera grasp: TF failed: {e}')
            return False

        p = handle_in_base.pose.position
        self.get_logger().info(
            f'Depth-camera grasp: handle at base_link ({p.x:.2f}, {p.y:.2f})')
        grasp_pos = np.array([p.x + _CALIB_X_M, p.y + _CALIB_Y_M, _GRASP_Z_M])

        grasp_joints = self._compute_ik(
            grasp_pos, preferred_seed=hover_joints, target_orientation=hover_orientation)
        if grasp_joints is None:
            self.get_logger().error('Depth-camera grasp: IK failed for grasp position')
            return False

        pick_up_pos = np.array([grasp_pos[0], grasp_pos[1], _APPROACH_Z_M + 0.05])
        pick_up_joints = self._compute_ik(pick_up_pos, preferred_seed=grasp_joints)
        if pick_up_joints is None:
            self.get_logger().error('Depth-camera grasp: IK failed for pick-up position')
            return False

        self._move_arm(grasp_joints)
        actual_pos = self._send_gripper(_GRIPPER_CLOSE)
        self._move_arm(pick_up_joints)
        return actual_pos

    # ------------------------------------------------------------------
    def _move_arm_hover(self, handle_base):
        """Move arm to fixed hover pose where camera faces down."""
        self._move_arm(_HOVER_JOINTS)
        return list(_HOVER_JOINTS)

    # ------------------------------------------------------------------
    # Ros2 action; 2 futures:
    # 1. `send_future` — resolves when the action server **accepts** the goal (quickly)
    # 2. `result_future` — resolves when the gripper **finishes** moving (takes time)
    # Returns the actual final joint position from the action result, used for _grasp_confirmed
    def _send_gripper(self, position) -> 'float | None':
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available after 5s')
            return None
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0 # max motor torque
        send_future = self._gripper_client.send_goal_async(goal)

        deadline = time.time() + 8.0
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            self.get_logger().error(f'Gripper goal send timed out (pos={position:.3f})')
            return None
        if not send_future.result().accepted:
            self.get_logger().error(f'Gripper goal rejected by server (pos={position:.3f})')
            return None

        result_future = send_future.result().get_result_async()
        deadline = time.time() + 10.0
        while not result_future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            self.get_logger().warn(f'Gripper result timed out (pos={position:.3f})')
            return None
        return result_future.result().result.position

    # ------------------------------------------------------------------
    # check if the gripper actually grasped a handle
    def _grasp_confirmed(self, actual_pos: 'float | None') -> bool:
        """Return True if the gripper stopped short enough to indicate a handle between the jaws."""
        if actual_pos is None:
            return False
        return actual_pos < _GRIPPER_CLOSE - _GRASP_DETECT_THRESHOLD

    # ------------------------------------------------------------------
    # drive backward at `_SERVO_BACKUP_VEL` m/s for `distance / speed` seconds
    def _backup(self, metres: float):
        """Drive the base straight back by the given distance, then stop."""
        twist = Twist()
        twist.linear.x = -_SERVO_BACKUP_VEL
        duration = metres / _SERVO_BACKUP_VEL
        t0 = time.time()
        while time.time() - t0 < duration:
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # the actual service callback
    def _grasp_cb(self, request, response):
        if not self._handle_history:
            response.success = False
            response.message = 'No handles detected — point camera at a handle first'
            return response

        # staleness check
        handle_age = time.time() - self._last_handle_time
        if handle_age > 3.0:
            response.success = False
            response.message = f'Handle detection is stale ({handle_age:.1f}s old) — re-point camera'
            return response

        # Average recent detections to reduce per-frame depth noise
        xs = [m.pose.position.x for m in self._handle_history]
        ys = [m.pose.position.y for m in self._handle_history]
        zs = [m.pose.position.z for m in self._handle_history]
        best = self._handle_history[-1]

        handle_stamped = PoseStamped()
        handle_stamped.header = best.header
        handle_stamped.header.stamp = rclpy.time.Time().to_msg()
        handle_stamped.pose = best.pose
        handle_stamped.pose.position.x = sum(xs) / len(xs)
        handle_stamped.pose.position.y = sum(ys) / len(ys)
        handle_stamped.pose.position.z = sum(zs) / len(zs)
        # averaged position transformed from `camera_depth_optical_frame` to `base_link` by TF2
        try:
            handle_in_base = self.tf_buffer.transform(
                handle_stamped, 'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            response.success = False
            response.message = f'TF transform to base_link failed: {e}'
            return response

        p = handle_in_base.pose.position
        self.get_logger().info(
            f'Handle at base_link ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

        handle_base = np.array([p.x + _CALIB_X_M, p.y + _CALIB_Y_M, 0.0])

        # 1. Open gripper
        self._send_gripper(_GRIPPER_OPEN)

        # 2. Move arm to hover pose (high above handle, wrist angle for camera-down)
        hover_joints = self._move_arm_hover(handle_base)
        if hover_joints is None:
            response.success = False
            response.message = 'IK failed for hover position — handle may be out of reach'
            return response

        for attempt in range(_GRASP_MAX_RETRIES + 1):
            # 3. Move robot base to centre handle in top-down gripper camera view
            if not self._visual_servo_base():
                if self._servo_exit_reason == 'depth_available':
                    actual_pos = self._depth_camera_grasp(hover_joints)
                    if actual_pos is None:
                        response.success = False
                        response.message = 'Depth-camera grasp fallback failed'
                        return response
                    if self._grasp_confirmed(actual_pos):
                        self._grasp_proceed_pub.publish(Bool(data=True))
                        response.success = True
                        response.message = 'Grasped via depth-camera fallback (visual servo gave up)'
                        return response
                    # Slip on depth-camera path — back up and retry
                    self.get_logger().warn(
                        f'Depth-camera grasp: slip detected '
                        f'(pos={actual_pos:.3f}, attempt {attempt + 1}/{_GRASP_MAX_RETRIES + 1})')
                    self._send_gripper(_GRIPPER_OPEN)
                    self._backup(_GRASP_BACKUP_M)
                    self._move_arm_hover(handle_base)
                    continue
                elif self._servo_exit_reason in ('timeout', 'confirm_failed'):
                    self.get_logger().warn(
                        f'Visual servo failed ({self._servo_exit_reason}), '
                        f'attempt {attempt + 1}/{_GRASP_MAX_RETRIES + 1} — backing up and retrying')
                    self._backup(_GRASP_BACKUP_M)
                    self._move_arm_hover(handle_base)
                    continue
                else:
                    response.success = False
                    response.message = f'Base servo failed ({self._servo_exit_reason}) — grasp aborted'
                    return response

            # 3b. Nudge forward by _SERVO_FORWARD_OFFSET_M after convergence.
            # Loop-publish so the base controller doesn't time out the cmd_vel mid-nudge.
            if _SERVO_FORWARD_OFFSET_M > 0:
                nudge_twist = Twist()
                nudge_twist.linear.x = _SERVO_MIN_VEL
                t0 = time.time()
                nudge_dur = _SERVO_FORWARD_OFFSET_M / _SERVO_MIN_VEL
                while time.time() - t0 < nudge_dur:
                    self._cmd_vel_pub.publish(nudge_twist)
                    time.sleep(0.05)
                self._cmd_vel_pub.publish(Twist())

            # 4. Recover gripper world position via FK (base has moved since initial transform)
            fk_mat = self._chain.forward_kinematics(
                [0.0] + list(hover_joints) + [0.0, 0.0])
            gripper_base = _frame_to_base(fk_mat[:3, 3])
            grasp_pos = np.array([gripper_base[0], gripper_base[1], _GRASP_Z_M])

            # Pass hover orientation so IK keeps the gripper pointing straight down
            hover_orientation = fk_mat[:3, :3]
            grasp_joints = self._compute_ik(
                grasp_pos, preferred_seed=hover_joints, target_orientation=hover_orientation)
            if grasp_joints is None:
                response.success = False
                response.message = 'IK failed for grasp position after base servo'
                return response

            # 5. Lower arm to grasp height (same X/Y, lower Z)
            self._move_arm(grasp_joints)

            # 6. Close gripper and lift
            pick_up_pos = np.array([grasp_pos[0], grasp_pos[1], _APPROACH_Z_M + 0.1])
            pick_up_joints = self._compute_ik(pick_up_pos, preferred_seed=grasp_joints)
            if pick_up_joints is None:
                response.success = False
                response.message = 'IK failed for pick-up position'
                return response

            actual_pos = self._send_gripper(_GRIPPER_CLOSE)
            self._move_arm(pick_up_joints)

            # 7. Verify grasp — check if gripper stopped short of fully closing
            if self._grasp_confirmed(actual_pos):
                self._grasp_proceed_pub.publish(Bool(data=True))
                response.success = True
                response.message = (
                    f'Grasped handle at ({gripper_base[0]:.2f}, {gripper_base[1]:.2f}) in base_link')
                return response

            # Slip or miss — back up and retry the full servo cycle
            pos_str = f'{actual_pos:.3f}' if actual_pos is not None else 'unknown'
            self.get_logger().warn(
                f'Grasp slip detected (pos={pos_str}, expected <{_GRIPPER_CLOSE - _GRASP_DETECT_THRESHOLD:.3f}), '
                f'attempt {attempt + 1}/{_GRASP_MAX_RETRIES + 1}')
            self._send_gripper(_GRIPPER_OPEN)
            self._backup(_GRASP_BACKUP_M)
            self._move_arm_hover(handle_base)

        response.success = False
        response.message = (
            f'Grasp failed after {_GRASP_MAX_RETRIES + 1} attempts — handle may be unreachable or too slippery')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
