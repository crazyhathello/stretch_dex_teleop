import stretch_body.robot as rb
import numpy as np
import math
import time
import errno
from scipy.spatial.transform import Rotation
from stretch_body.robot_params import RobotParams
from hello_helpers import hello_misc as hm
import urchin as urdf_loader
import os
import simple_ik as si
import loop_timer as lt
import dex_teleop_parameters as dt
from multiprocessing import shared_memory
import pprint as pp
import robot_move as rm
from enum import Enum


def load_urdf(file_name):
    if not os.path.isfile(file_name):
        print()
        print('*****************************')
        print('ERROR: ' + file_name + ' was not found. Simple IK requires a specialized URDF saved with this file name. prepare_base_rotation_ik_urdf.py can be used to generate this specialized URDF.')
        print('*****************************')
        print()
        raise FileNotFoundError(
            errno.ENOENT, os.strerror(errno.ENOENT), file_name)
    urdf = urdf_loader.URDF.load(file_name, lazy_load_meshes=True)
    return urdf


def nan_in_configuration(configuration):
    for k, v in configuration.items():
        if math.isnan(v) or np.isnan(v):
            return True
    return False

class Mode(Enum):
    BASE = 1
    ARM = 2

####################################################
# CommandToLinearMotion, CommandToRotaryMotion 
#   (Copied from stretch_body/tools/bin/stretch_xbox_controller_teleop.py)
# 1. CommandToLinearMotion: Translate y-axis movement of stick to base forward/backward movement
# 2. CommandToRotaryMotion: Translate x-axis movement of stick to base rotation direction
####################################################
class CommandToLinearMotion():
    def __init__(self, command_dead_zone_value, move_duration_s, max_distance_m, accel_m):
        # This expects
        # a command value with a magnitude between 0.0 and 1.0, inclusive
        # a dead_zone with a magnitude greater than or equal to 0.0 and less than 1.0

        self.dead_zone = abs(command_dead_zone_value)
        self.move_duration_s = abs(move_duration_s)
        self.max_distance_m = abs(max_distance_m)
        self.accel_m = abs(accel_m)

        # check that the values are reasonable
        assert self.dead_zone >= 0.0
        assert self.dead_zone <= 0.9, 'WARNING: CommandToLinearMotion.__init__ command_dead_zone_value is strangely large command_dead_zone_value = abs({0}) > 0.9.'.format(
            command_dead_zone_value)
        assert self.move_duration_s > 0.01, 'WARNING: CommandToLinearMotion.__init__ move_duration_s = abs({0}) <= 0.01 seconds, which is a short time for a single move.'.format(
            move_duration_s)
        assert self.move_duration_s <= 1.0, 'WARNING: CommandToLinearMotion.__init__ move_duration_s = abs({0}) > 1.0 seconds, which is a long time for a single move.'.format(
            move_duration_s)
        assert self.max_distance_m <= 0.3, 'WARNING: CommandToLinearMotion.__init__ max_distance_m = abs({0}) > 0.3 meters, which is a long distance for a single move.'.format(
            max_distance_m)
        assert self.accel_m <= 30.0, 'WARNING: CommandToLinearMotion.__init__ accel_m = abs({0}) > 30.0 m/s^2, which is very high (> 3 g).'.format(
            accel_m)

    def get_dist_vel_accel(self, output_sign, command_value):
        # Larger commands attempt to move over larger distances in the
        # same amount of time by moving at higher velocities.
        c_val = abs(command_value)

        assert c_val <= 1.0, 'ERROR: CommandToLinearMotion.get_dist_vel_accel given command value > 1.0, command_value = {0}'.format(
            command_value)
        assert c_val > self.dead_zone, 'ERROR: CommandToLinearMotion.get_dist_vel_accel the command should not be executed due to its value being within the dead zone: abs(command_value) = abs({0}) <= {1} = self.dead_zone'.format(
            command_value, self.dead_zone)
        if 1:
            scale = (c_val - self.dead_zone) / (1.0 - self.dead_zone) ** 2
        else:
            scale = c_val - self.dead_zone
        d_m = (scale * (self.max_distance_m / (1.0 - self.dead_zone)))
        d_m = math.copysign(d_m, output_sign)
        v_m = d_m / self.move_duration_s  # average m/s for a move of distance d_m to last for time move_s
        a_m = self.accel_m
        return d_m, v_m, a_m

class CommandToRotaryMotion():
    def __init__(self, command_dead_zone_value, move_duration_s, max_angle_rad, accel_rad):
        # This expects
        # a command value with a magnitude between 0.0 and 1.0, inclusive
        # a dead_zone with a magnitude greater than or equal to 0.0 and less than 1.0

        self.dead_zone = abs(command_dead_zone_value)
        self.move_duration_s = abs(move_duration_s)
        self.max_angle_rad = abs(max_angle_rad)
        self.accel_rad = abs(accel_rad)

        # check that the values are reasonable
        assert self.dead_zone >= 0.0
        assert self.dead_zone <= 0.9, 'WARNING: CommandToRotaryMotion.__init__ command_dead_zone_value is strangely large command_dead_zone_value = abs({0}) > 0.9.'.format(
            command_dead_zone_value)
        assert self.move_duration_s > 0.01, 'WARNING: CommandToRotaryMotion.__init__ move_duration_s = abs({0}) <= 0.01 second, which is a short time for a single move.'.format(
            move_duration_s)
        assert self.move_duration_s <= 1.0, 'WARNING: CommandToRotaryMotion.__init__ move_duration_s = abs({0}) > 1.0 second, which is a long time for a single move.'.format(
            move_duration_s)
        assert self.max_angle_rad <= 0.7, 'WARNING: CommandToRotaryMotion.__init__ max_angle_rad = abs({0}) > 0.7 , which is a large angle for a single move (~40.0 deg).'.format(
            max_angle_rad)
        assert self.accel_rad <= 4.0 * 10, 'WARNING: CommandToRotaryMotion.__init__ accel_rad = abs({0}) > 4.0 rad/s^2, which is high.'.format(
            accel_rad)

    def get_dist_vel_accel(self, output_sign, command_value):
        # Larger commands attempt to move over larger distances in the
        # same amount of time by moving at higher velocities.
        c_val = abs(command_value)
        assert c_val <= 1.0, 'ERROR: CommandToRotaryMotion.get_dist_vel_accel given command value > 1.0, command_value = {0}'.format(
            command_value)
        assert c_val > self.dead_zone, 'ERROR: CommandToRotaryMotion.get_dist_vel_accel the command should not be executed due to its value being within the dead zone: abs(command_value) = abs({0}) <= {1} = self.dead_zone'.format(
            command_value, self.dead_zone)

        scale = c_val - self.dead_zone
        d_r = (scale * (self.max_angle_rad / (1.0 - self.dead_zone)))
        d_r = math.copysign(d_r, output_sign)
        v_r = d_r / self.move_duration_s  # average m/s for a move of distance d_m to last for time move_s
        a_r = self.accel_rad
        return d_r, v_r, a_r

####################################################
# GripperToGoal - Main class responsible for AR teleop logic
####################################################
class GripperToGoal:
    def __init__(self, robot_speed, starting_configuration, robot_allowed_to_move, using_stretch_2):
        if using_stretch_2:
            self.grip_range = dt.dex_wrist_grip_range
        else:
            self.grip_range = dt.dex_wrist_3_grip_range

        self.using_stretch_2 = using_stretch_2

        self.joints_allowed_to_move = ['stretch_gripper', 'joint_arm_l0', 'joint_lift',
                                       'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_mobile_base_rotate_by']

        # Get Wrist URDF joint limits
        rotary_urdf_file_name = './stretch_base_rotation_ik_with_fixed_wrist.urdf'
        rotary_urdf = load_urdf(rotary_urdf_file_name)
        wrist_joints = ['joint_wrist_yaw',
                        'joint_wrist_pitch', 'joint_wrist_roll']
        self.wrist_joint_limits = {}
        for joint_name in wrist_joints:
            joint = rotary_urdf.joint_map.get(joint_name, None)
            if joint is not None:
                lower = float(joint.limit.lower)
                upper = float(joint.limit.upper)
                self.wrist_joint_limits[joint.name] = (lower, upper)

        self.robot_allowed_to_move = robot_allowed_to_move
        self.drop_extreme_wrist_orientation_change = True

        # Initialize the filtered wrist orientation that is used to
        # command the robot. Simple exponential smoothing is used to
        # filter wrist orientation values coming from the interface
        # objects.
        self.filtered_wrist_orientation = np.array([0.0, 0.0, 0.0])

        # Initialize the filtered wrist position that is used to command
        # the robot. Simple exponential smoothing is used to filter wrist
        # position values coming from the interface objects.
        self.filtered_wrist_position_configuration = np.array([
            starting_configuration['joint_lift'],
            starting_configuration['joint_arm_l0']
        ])

        self.prev_commanded_wrist_orientation = {'joint_wrist_yaw': None,
                                                 'joint_wrist_pitch': None,
                                                 'joint_wrist_roll': None}

        # This is the weight multiplied by the current wrist angle command when performing exponential smoothing.
        # 0.5 with 'max' robot speed was too noisy on the wrist
        # self.wrist_orientation_filter = dt.exponential_smoothing_for_orientation
        self.wrist_orientation_filter = 0.4

        # This is the weight multiplied by the current wrist position command when performing exponential smoothing.
        # commands before sending them to the robot
        self.wrist_position_filter = dt.exponential_smoothing_for_position

        # Initialize IK
        self.simple_ik = si.SimpleIK()

        ##########################################################
        # Prepare the robot last to avoid errors due to blocking calls
        # associated with other aspects of setting things up, such as
        # initializing SimpleIK.
        self.robot = rb.Robot()
        self.robot.startup()

        print('stretch_body file imported =', rb.__file__)
        transport_version = self.robot.arm.motor.transport.version
        print('stretch_body using transport version =', transport_version)

        self.robot_move = rm.RobotMove(self.robot, speed=robot_speed)
        self.robot_move.print_settings()

        self.robot_move.to_configuration(
            starting_configuration, speed='default')
        self.robot.push_command()
        self.robot.wait_command()

        # Set the current mobile base angle to be 0.0 radians.
        self.robot.base.reset_odometry()
        ##########################################################

        self.print_robot_status_thread_timing = False
        self.debug_wrist_orientation = True

        self.max_allowed_wrist_yaw_change = dt.max_allowed_wrist_yaw_change
        self.max_allowed_wrist_roll_change = dt.max_allowed_wrist_roll_change
        
        # Robot mode status
        self.operation_mode = Mode.ARM

        # Grip status
        self.grip_status = 0 # -95 (fully open) ~ +95 (closed tight)

        self.last_controller_pos = None # [origin lift height, origin arm extension]

    def __del__(self):
        print('GripperToGoal.__del__: stopping the robot')
        self.robot.stop()


    def get_rotation(self, mock_configuration):
        '''
        Added function to convert quaternion form rotation of Quest Controller to wrist rotation
        After converting quaternion to euler form, wrist_roll and wrist_yall are adjusted to match controller input

        Input:
            mock_configuration: dictionary
            the values of quaternion are (mock_configuration['q1'] to mock_configuration['q4])

        Output:
            wrist_yaw, wrist_roll, wrist_pitch
        '''
        q1, q2, q3, q4 = mock_configuration['q1'], mock_configuration[
            'q2'], mock_configuration['q3'], mock_configuration['q4']

        # print([q1,q2,q3,q4])
        r = Rotation.from_quat([q1, q2, q3, q4])
        ypr = r.as_euler('ZXY', degrees=False)

        wrist_roll = hm.angle_diff_rad(ypr[0] + np.pi, 0.0)

        # wrist_joint_limits: the upper and lower bound of valid rotations
        # {'joint_wrist_yaw': (-1.75, 4.0), 'joint_wrist_pitch': (-1.2566370614359172, 0.56), 'joint_wrist_roll': (-3.14, 3.14)}

        wrist_roll = -1*wrist_roll + np.pi

        # adjust the calculated wrist_roll to fit the range(-3.14, 3.14)
        # i.e. turning pi+x degree clockwise = pi -x degree counterclockwise = -pi +x degree clockwise(pi+x - 2*pi) and vice versa
        if wrist_roll > 3.14:
            wrist_roll -= 2*3.14

        if wrist_roll < -3.14:
            wrist_roll += 2*3.14

        wrist_pitch = -1*ypr[1]

        wrist_yaw = hm.angle_diff_rad(ypr[2] + np.pi, 0.0)

        wrist_yaw = -1*wrist_yaw - np.pi

        return wrist_yaw, wrist_roll, wrist_pitch

    def new_update_goal(self, mock_configuration):
        '''
        This method is modified from update_goal in mainly two aspects:
            1. Removed usage of IK:
                Originally, the method reads in (x,y,z) position and compute 'joint_mobile_base_rotation', 'joint_lift', 'joint_arm_l0' using IK
                Right now, we fix the base rotation and directly calculates relative lift and arm of controllers to start position

            2. Changed method for rotation calculation:
                Originally, the rotation is read in as rotation matrix and converted to euler form to calculate values for roll, yaw, pitch
                In our method, rotation is read in as quaternion, so we converted quaternion to euler for calculation, please refer to get_rotation for details

        Input:
            mock_configuration: 
                dictionary with following keys:
                    'joint_mobile_base_rotation', 'joint_lift', 'joint_arm_l0' -> wrist position
                    'stretch_gripper',
                    'q1', 'q2', 'q3', 'q4' -> wrist rotation
        '''  

        new_goal_configuration = mock_configuration

        if new_goal_configuration is None:
            print(f"WARNING: Configuration specified is None.")
        else:
            
            right_button_a = new_goal_configuration['right_button_a']
            right_button_b = new_goal_configuration['right_button_b']
            right_thumbstick_x = new_goal_configuration['right_thumbstick_x']
            right_thumbstick_y = new_goal_configuration['right_thumbstick_y']
            right_safety = new_goal_configuration['right_safety']
            left_safety = new_goal_configuration['left_safety']

            # Initialize origin of movement on startup
            if self.last_controller_pos is None:
                self.last_controller_pos = [new_goal_configuration["joint_lift"], new_goal_configuration["joint_arm_l0"]]
            
            # Handle safety lock
            if right_safety>0.8 and left_safety>0.8:
                if not self.robot_allowed_to_move:
                    self.last_controller_pos = [new_goal_configuration["joint_lift"],new_goal_configuration["joint_arm_l0"]]
                self.robot_allowed_to_move = True
            else:
                self.robot_allowed_to_move = False
            
            # Return if safety lock
            if not self.robot_allowed_to_move:
                return
            
            controller_delta = [new_goal_configuration["joint_lift"]- self.last_controller_pos[0],new_goal_configuration["joint_arm_l0"]-self.last_controller_pos[1]]
            
            # scaled the delta by 2.5 
            self.filtered_wrist_position_configuration = [self.filtered_wrist_position_configuration[0]+ 2.5*self.wrist_position_filter*controller_delta[0],
                                                          self.filtered_wrist_position_configuration[1]+ 2.5*self.wrist_position_filter*controller_delta[1]]

            self.last_controller_pos = [new_goal_configuration["joint_lift"],new_goal_configuration["joint_arm_l0"]]

            # new_wrist_position_configuration = np.array([new_goal_configuration['joint_mobile_base_rotation'],
            #                                     new_goal_configuration['joint_lift']-self.origin_of_movement[0],
            #                                     new_goal_configuration['joint_arm_l0']-self.origin_of_movement[1]])
            if right_button_a and not right_button_b:
                self.operation_mode = Mode.ARM
                self.robot.pimu.trigger_beep()
                self.robot.push_command()
            elif right_button_b and not right_button_a:
                self.operation_mode = Mode.BASE
                self.robot.pimu.trigger_beep()
                self.robot.push_command()
                time.sleep(0.3)
                self.robot.pimu.trigger_beep()
                self.robot.push_command()
            
            if self.operation_mode == Mode.BASE:
                # ######################### BASE ########################################
                # Regular Motion
                dead_zone = 0.1  # 0.25 #0.1 #0.2 #0.3 #0.4
                move_s = 0.6
                max_dist_m = 0.06  # 0.04 #0.05
                accel_m = 0.2  # 0.1
                command_to_linear_motion = CommandToLinearMotion(dead_zone, move_s, max_dist_m, accel_m)

                move_s = 0.05
                max_dist_rad = 0.10  # 0.2 #0.25 #0.1 #0.09
                accel_rad = 0.8  # 0.05
                command_to_rotary_motion = CommandToRotaryMotion(dead_zone, move_s, max_dist_rad, accel_rad)

                ############################
                # Fast Motion
                fast_move_s = 0.6
                fast_max_dist_m = 0.12
                fast_accel_m = 0.8
                # fast, but unstable on thresholds: 0.6 s, 0.15 m, 0.8 m/s^2

                fast_command_to_linear_motion = CommandToLinearMotion(dead_zone, fast_move_s, fast_max_dist_m, fast_accel_m)
                fast_move_s = 0.2
                fast_max_dist_rad = 0.6
                fast_accel_rad = 0.8
                fast_command_to_rotary_motion = CommandToRotaryMotion(dead_zone, fast_move_s, fast_max_dist_rad, fast_accel_rad)

                # Manage base
                forward_command = right_thumbstick_y
                turn_command = right_thumbstick_x

                fast_navigation_mode = False
                # navigation_mode_trigger = controller_state['right_trigger_pulled']
                # if (navigation_mode_trigger > 0.5):
                #     fast_navigation_mode = True

                ##################
                # convert robot commands to robot movement
                # only allow a pure translation or a pure rotation command
                if abs(forward_command) > abs(turn_command):
                    if abs(forward_command) > dead_zone:
                        output_sign = math.copysign(1, forward_command)
                        if not fast_navigation_mode:
                            d_m, v_m, a_m = command_to_linear_motion.get_dist_vel_accel(output_sign, forward_command)
                        else:
                            d_m, v_m, a_m = fast_command_to_linear_motion.get_dist_vel_accel(output_sign, forward_command)
                        self.robot.base.translate_by(d_m, v_m, a_m)
                        self.robot.push_command()

                else:
                    if abs(turn_command) > dead_zone:
                        output_sign = -math.copysign(1, turn_command)
                        if not fast_navigation_mode:
                            d_rad, v_rad, a_rad = command_to_rotary_motion.get_dist_vel_accel(output_sign, turn_command)
                        else:
                            d_rad, v_rad, a_rad = fast_command_to_rotary_motion.get_dist_vel_accel(output_sign, turn_command)
                        self.robot.base.rotate_by(d_rad, v_rad, a_rad)
                        self.robot.push_command()


            elif self.operation_mode == Mode.ARM:
                # Use exponential smoothing to filter the wrist
                # position configuration used to command the
                # robot.
                # self.filtered_wrist_position_configuration = (((1.0 - self.wrist_position_filter) * self.filtered_wrist_position_configuration) +
                #                                             (self.wrist_position_filter * new_wrist_position_configuration))

                # Original formula: (1-float)*old + float*new = (1-float)*old + float*(old + delta) = old + float*delta => new formula
                # print("FILTERED_WRIST_POSITION_CONFIG:",self.filtered_wrist_position_configuration)
                new_goal_configuration['joint_lift'] = self.filtered_wrist_position_configuration[0]
                new_goal_configuration['joint_arm_l0'] = self.filtered_wrist_position_configuration[1]

                self.simple_ik.clip_with_joint_limits(new_goal_configuration)

                print("joint_lift: ", new_goal_configuration["joint_lift"])
                print("joint_arm_l0: ", new_goal_configuration["joint_arm_l0"])


                #################################
                

                #################################
                # Adjust grip width based on 

                # Initial config is specified in self.grip_status in init()
                # -95 is tightest, +95 is loosest
                
                if new_goal_configuration["right_trigger_status"] > 0.8:
                    self.grip_status -= 5 # Tighter
                if new_goal_configuration["left_trigger_status"] > 0.8:
                    self.grip_status += 5 # Looser
                self.grip_status = np.clip(self.grip_status, -95,95)
                new_goal_configuration["stretch_gripper"] = self.grip_status

                ##################################################
                # INPUT: x_axis, y_axis, z_axis

                wrist_yaw, wrist_roll, wrist_pitch = self.get_rotation(
                    mock_configuration)

                lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_yaw']
                if (wrist_yaw < lower_limit):
                    wrist_yaw = wrist_yaw + (2.0*np.pi)

                if self.debug_wrist_orientation:
                    print('___________')
                    print('wrist_yaw, wrist_pitch, wrist_roll = {:.2f}, {:.2f}, {:.2f} deg'.format((180.0 * (wrist_yaw/np.pi)),
                                                                                                (180.0 * (wrist_pitch/np.pi)),
                                                                                                (180.0 * (wrist_roll/np.pi))))

                limits_violated = False
                lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_yaw']
                if (wrist_yaw < lower_limit) or (wrist_yaw > upper_limit):
                    limits_violated = True
                lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_pitch']
                if (wrist_pitch < lower_limit) or (wrist_pitch > upper_limit):
                    limits_violated = True
                lower_limit, upper_limit = self.wrist_joint_limits['joint_wrist_roll']
                if (wrist_roll < lower_limit) or (wrist_roll > upper_limit):
                    limits_violated = True

                ################################################################
                # DROP GRIPPER ORIENTATION GOALS WITH LARGE JOINT ANGLE CHANGES
                #
                # Dropping goals that result in extreme changes in joint
                # angles over a single time step avoids the nearly 360
                # degree rotation in an opposite direction of motion that
                # can occur when a goal jumps across a joint limit for a
                # joint with a large range of motion like the roll joint.
                #
                # This also reduces the potential for unexpected wrist
                # motions near gimbal lock when the yaw and roll axes are
                # aligned (i.e., the gripper is pointed down to the
                # ground). Goals representing slow motions that traverse
                # near this gimbal lock region can still result in the
                # gripper approximately going upside down in a manner
                # similar to a pendulum, but this results in large yaw
                # joint motions and is prevented at high speeds due to
                # joint angles that differ significantly between time
                # steps. Inverting this motion must also be performed at
                # low speeds or the gripper will become stuck and need to
                # traverse a trajectory around the gimbal lock region.
                #
                extreme_difference_violated = False
                if self.drop_extreme_wrist_orientation_change:
                    prev_wrist_yaw = self.prev_commanded_wrist_orientation['joint_wrist_yaw']
                    if prev_wrist_yaw is not None:
                        diff = abs(wrist_yaw - prev_wrist_yaw)
                        if diff > self.max_allowed_wrist_yaw_change:
                            print('extreme wrist_yaw change of {:.2f} deg'.format(
                                (180.0 * (diff/np.pi))))
                            extreme_difference_violated = True
                    prev_wrist_roll = self.prev_commanded_wrist_orientation['joint_wrist_roll']
                    if prev_wrist_roll is not None:
                        diff = abs(wrist_roll - prev_wrist_roll)
                        if diff > self.max_allowed_wrist_roll_change:
                            print('extreme wrist_roll change of {:.2f} deg'.format(
                                (180.0 * (diff/np.pi))))
                            extreme_difference_violated = True
                #
                ################################################################

                if self.debug_wrist_orientation:
                    if limits_violated:
                        print('The wrist angle limits were violated.')
                        print((wrist_yaw, wrist_roll, wrist_pitch))

                if (not extreme_difference_violated) and (not limits_violated):
                    new_wrist_orientation = np.array(
                        [wrist_yaw, wrist_pitch, wrist_roll])

                    # Use exponential smoothing to filter the wrist
                    # orientation configuration used to command the
                    # robot.
                    self.filtered_wrist_orientation = (((1.0 - self.wrist_orientation_filter) * self.filtered_wrist_orientation) +
                                                    (self.wrist_orientation_filter * new_wrist_orientation))

                    new_goal_configuration['joint_wrist_yaw'] = self.filtered_wrist_orientation[0]
                    new_goal_configuration['joint_wrist_pitch'] = self.filtered_wrist_orientation[1]
                    new_goal_configuration['joint_wrist_roll'] = self.filtered_wrist_orientation[2]

                    self.prev_commanded_wrist_orientation = {'joint_wrist_yaw': self.filtered_wrist_orientation[0],
                                                            'joint_wrist_pitch': self.filtered_wrist_orientation[1],
                                                            'joint_wrist_roll': self.filtered_wrist_orientation[2]}

                # Convert from the absolute goal for the mobile
                # base to an incremental move to be performed
                # using rotate_by. This should be performed just
                # before sending the commands to make sure it's
                # using the most rececnt mobile base angle
                # estimate to reduce overshoot and other issues.

                # convert base odometry angle to be in the range -pi to pi
                # negative is to the robot's right side (counterclockwise)
                # positive is to the robot's left side (clockwise)
                # base_odom_theta = hm.angle_diff_rad(
                #     self.robot.base.status['theta'], 0.0)
                # current_mobile_base_angle = base_odom_theta

                base_rotation_speed = 3.14/50
                if new_goal_configuration["right_thumbstick_x"]>0.5:
                    new_goal_configuration['joint_mobile_base_rotate_by'] = -base_rotation_speed
                elif new_goal_configuration["right_thumbstick_x"] <-0.5:
                    new_goal_configuration['joint_mobile_base_rotate_by'] = base_rotation_speed


                del new_goal_configuration['joint_mobile_base_rotation']

                # If motion allowed, command the robot to move to the target configuration
                if self.robot_allowed_to_move:
                    if nan_in_configuration(new_goal_configuration):
                        print()
                        print(
                            '******************************************************************')
                        print(
                            'WARNING: dex_teleop: new_goal_configuration has a nan, so skipping execution on the robot')
                        print()
                        print('     new_goal_configuration =',
                            new_goal_configuration)
                        print()
                        print(
                            '******************************************************************')
                        print()
                    else:
                        # print("NEW_GOAL_CONFIG: ",new_goal_configuration)
                        self.robot_move.to_configuration(
                            new_goal_configuration, self.joints_allowed_to_move)
                        self.robot.push_command()
                else:
                    print("SAFETY_LOCK_ENGAGED: Robot cannot move due to safety lock. Please hold down safety triggers on both controllers to resume control.")

                # Print robot status timing stats, if desired.
                if self.print_robot_status_thread_timing:
                    self.robot.non_dxl_thread.stats.pretty_print()
                    print()
                    self.robot.dxl_end_of_arm_thread.stats.pretty_print()
                    print()
                    self.robot.dxl_head_thread.stats.pretty_print()
                    print()
                    self.robot.sys_thread.stats.pretty_print()


##############################################################
# NOTES
##############################################################

#######################################
#
# Overview
#
# Dexterous teleoperation uses a marker dictionary representing either
# a real or virtual ArUco marker specified with respect to the
# camera's frame of reference. The marker's position controls the
# robot's wrist position via inverse kinematics (IK). The marker's
# orientation directly controls the joints of the robot's dexterous
# wrist.
#
#######################################

#######################################
#
# The following coordinate systems are important to this teleoperation
# code
#
#######################################

#######################################
# Camera Coordinate System
#
# Camera on the floor looking with the top of the camer facing away
# from the person.
#
# This configuration matches the world frame's coordinate system with
# a different origin that is mostly just translated along the x and y
# axes.
#
# Origin likely at the optical cemter of a pinhole
# model of the camera.
#
# The descriptions below describe when the robot's mobile base is at
# theta = 0 deg.
#
# x-axis
# human left is pos / robot forward is pos
# human right is neg / robot backward is neg

# y-axis
# human arm extended is neg / robot arm extended is neg
# human arm retracted is pos / robot arm retracted is pos

# z-axis
# up is positive for person and the robot
# down is negative for person and the robot
#
#######################################

#######################################
# IK World Frame Coordinate System
#
# Origin at the axis of rotation of the mobile
# base on the floor.
#
# x-axis
# human/robot left is pos
# human/robot right is neg

# y-axis
# human/robot forward is neg
# human/robot backward is pos

# z-axis
# human/robot up is pos
# human/robot down is neg
#
#######################################

#######################################
# Robot Wrist Control

# wrist yaw
#     - : deployed direction
#     0 : straight out parallel to the telescoping arm
#     + : stowed direction

# wrist pitch
#     - : up
#     0 : horizontal
#     + : down

# wrist roll
#     - :
#     0 : horizontal
#     + :
#
#######################################

##############################################################
