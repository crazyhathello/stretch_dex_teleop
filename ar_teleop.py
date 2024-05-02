'''
This is our current implementation which consists of:
    1. set up ZMQ connection
    2. within the while True loop:
        a. read controller information from ZMQ
        b. set up the initial position of controller on first iteration
        c. for each later iteration:
            calculate relative position to initial position
            generate new configuration
            call gripper_to_goal.new_update_goal to execute the new goal

We modified the original update_goal method to fit our new configuration format
'''
import gripper_to_goal_custom as gg
import dex_teleop_parameters as dt
import numpy as np
import time
from test_parse_controller_input import get_controller_data_all
import zmq
import json
import argparse


context = zmq.Context()
socket = context.socket(zmq.PULL)

# Quest 3 IP on CMU-DEVICE
socket.connect("tcp://172.26.187.122:12345")

# Quest 3 IP on Jensen's WIFI
# socket.connect("tcp://192.168.1.179:12345")

# robot_speed = "slow"
robot_speed = "fastest_stretch_2" 
manipulate_on_ground = False
robot_allowed_to_move = True
using_stretch_2 = True


gripper_to_goal = gg.GripperToGoal(robot_speed,
    dt.get_starting_configuration(
        dt.get_lift_middle(manipulate_on_ground)),
    robot_allowed_to_move,
    using_stretch_2
    )

grip_width_start = 50
base_rotation = 0
# controller_movement = get_controller_data_all()
# for lift_delta, arm_ext_delta in controller_movement:
#     # time.sleep(0.01)
initial_configuration = {
    'joint_mobile_base_rotation': base_rotation,
    'joint_lift': 0.6,
    'right_trigger_status': 0, # 0~1
    'left_trigger_status': 0, # 0~1
    'right_safety': 1, # 0~1
    'left_safety': 1, # 0~1
    'joint_arm_l0': 0.25,
    'q1':-0.04,
    'q2':0.19,
    'q3':-0.08,
    'q4':0.97,
    'right_thumbstick_x':0,
    'right_thumbstick_y':0,
    'right_button_a': False,
    'right_button_b': False
}
gripper_to_goal.new_update_goal(initial_configuration)
time.sleep(5)

# starting_controller_y = 0
# starting_controller_z = 0
# started = False
starting_rotation = [0.0, 0.0, 0.0, 1.0]

interval = 0
while True:
    # Receive message from remote server
    # time.sleep(0.01)
    message = socket.recv()
    try:
        data = json.loads(message.decode())
    except json.JSONDecodeError:
        print("Terminating the connection...")
        socket.close()
        context.term()
        break

    if interval%10 == 0:
        # Deserialize the JSON message
        right_controller = data["RightController"]
        left_controller = data["LeftController"]
        xyz = right_controller['RightLocalPosition']
        right_trigger_status = right_controller["RightIndexTrigger"]
        left_trigger_status = left_controller["LeftIndexTrigger"]
        right_safety = right_controller["RightHandTrigger"]
        left_safety = left_controller["LeftHandTrigger"]
        right_thumbstick_x, right_thumbstick_y = [float(x) for x in right_controller['RightThumbstickAxes'].split(',')]
        right_button_a = right_controller["RightA"]
        right_button_b = right_controller["RightB"]

        print("Right_thumbstick_x: ",right_thumbstick_x)
        
        controller_rotation = [float(x) for x in right_controller['RightLocalRotation'].split(',')]
        
        base_rotation_speed = 3.14/100
        if right_thumbstick_x > 0.5:
            base_rotation -= base_rotation_speed
        elif right_thumbstick_x < -0.5:
            base_rotation += base_rotation_speed

        # if not started:
        #     _, y_str, z_str = xyz.split(',')
        #     starting_controller_y = float(y_str)
        #     starting_controller_z = float(z_str)
        #     started = True

        _, y_str, z_str = xyz.split(',')
        # y = float(y_str) - starting_controller_y
        # z = float(z_str) - starting_controller_z
        # print(y,z)
        
        # if y==0.0 and z ==0.0:
        #     y = starting_controller_y
        #     z = starting_controller_z
        lift_pos = float(y_str)
        arm_ext_pos = float(z_str)
    
        mock_configuration = {
            'joint_mobile_base_rotation': base_rotation ,
            'joint_lift': lift_pos,
            'right_trigger_status': right_trigger_status, # 0~1
            'left_trigger_status': left_trigger_status, # 0~1
            'right_safety': right_safety, # 0~1
            'left_safety': left_safety, # 0~1
            'joint_arm_l0': arm_ext_pos,
            'q1': controller_rotation[0],
            'q2': controller_rotation[1],
            'q3': controller_rotation[2],
            'q4': controller_rotation[3],
            'right_thumbstick_x':right_thumbstick_x,
            'right_thumbstick_y':right_thumbstick_y,
            'right_button_a': right_button_a,
            'right_button_b': right_button_b
        }
        gripper_to_goal.new_update_goal(mock_configuration)
    interval+=1

# mock_configuration = {
#             'joint_mobile_base_rotation': 0.0,
#             'joint_lift': 0.6 ,
#             'stretch_gripper': grip_width_start, #-100~+100
#             'joint_arm_l0': 0.25,
#             'joint_wrist_yaw': 0.0 * np.pi,
#             'joint_wrist_pitch': 0.0 * np.pi,
#             'joint_wrist_roll': 0.0 * np.pi
#         }
# mock_configuration = {
#             'joint_mobile_base_rotation': 0.0,
#             'joint_lift': 0.6 ,
#             'stretch_gripper': grip_width_start, #-100~+100
#             'joint_arm_l0': 0.25,
#             'q1': 0.0,
#             'q2': 0.0,
#             'q3': 0.0,
#             'q4': 1.0
#         }
# gripper_to_goal.new_update_goal(mock_configuration)
# print("Exiting after 10 seconds...")
# time.sleep(10)

del gripper_to_goal