import gripper_to_goal_custom as gg
import dex_teleop_parameters as dt
import numpy as np
import time

manipulate_on_ground = False
robot_allowed_to_move = True
using_stretch_2 = True

gripper_to_goal = gg.GripperToGoal('slow',
    dt.get_starting_configuration(
        dt.get_lift_middle(manipulate_on_ground)),
    robot_allowed_to_move,
    using_stretch_2
    )

# Wait for starting configuration
#time.sleep(10)

# goal_dict={
#         "grip_width": 0.05,
#         "wrist_position": np.array([0.4,0.0,0.5]),
#         "gripper_x_axis": np.array([0.0,0.0,0.0]),
#         "gripper_y_axis": np.array([0.0,0.0,0.0]),
#         "gripper_z_axis": np.array([0.0,0.0,0.0])
#     }
# gripper_to_goal.update_goal(**goal_dict)



mock_configuration = {
    'joint_mobile_base_rotation': 0.0,
    'joint_lift': 0.77,
    'stretch_gripper': -90, #-100~+100
    'joint_arm_l0': 0.05,
    'joint_wrist_yaw': 0.9 * np.pi,
    'joint_wrist_pitch': 0.0 * np.pi,
    'joint_wrist_roll': 0.0 * np.pi
}
gripper_to_goal.new_update_goal(mock_configuration)


time.sleep(10)

del gripper_to_goal