'''
This is our current implementation which consists of:
    1. set up ZMQ connection
    2. set up the initial position of controller on first iteration
    3. within the while True loop:
        a. read controller information from ZMQ
        b. generate new configuration
           call gripper_to_goal.update_goal to execute the new goal

We modified the original update_goal method to fit our new configuration format
'''

import controller_to_robot as c2r
import dex_teleop_parameters as dt
import time
import zmq
import json
import argparse

'''
Parse the command line argument
1. The argument -i is the interval of responding the controller data
2. The argument -s is the speed of the robot, the choices include slow or fastest_stretch_2
'''
parser = argparse.ArgumentParser()
parser.add_argument('-i', '--interval', type=int, 
                    default=10, help='Interval in seconds')
parser.add_argument('-s', '--speed', choices=['slow', 'fastest_stretch_2'],
                    default='fastest_stretch_2', help='Speed option (choices: slow, fastest_stretch_2)')
args = parser.parse_args()

''' ZMQ coniguration'''
context = zmq.Context()
socket = context.socket(zmq.PULL)

''' Quest configuration'''
# Quest 3 IP on CMU-DEVICE
socket.connect("tcp://172.26.172.110:12345")
# Quest 3 IP on Jensen's WIFI
# socket.connect("tcp://192.168.1.179:12345")

''' Robot configuration'''
robot_speed = args.speed
manipulate_on_ground = False
robot_allowed_to_move = True
using_stretch_2 = True

'''Call the gripper to goal class for AR teleop logic with the robot configuration'''
gripper_to_goal = c2r.GripperToGoal(robot_speed,
    dt.get_starting_configuration(dt.get_lift_middle(manipulate_on_ground)),
    robot_allowed_to_move,
    using_stretch_2
)

'''
Set up the initial configuration of the update goal
This initial movement is just to demonstrate the script has launched, 
    the robot can move, and to move the arm to a good initial position.
The argument are set based on user experience

1. joint_lift is set to 0.6, which is similar to the position of human
2. the right and left trigger are set to 0, to make sure the initialization of gripper remain the same place
3. the right safety and left safety are set to 1 to enable the safety lock so robot is able to move to initial position
4. joint_arm_10 is set to 
5. q1, q2, q3, q4 are the quaternion form
6. right_thumbstick_x is the base rotation, it is set to 0 in the initial configuration
7. right_thumbstick_y is the base translation, it is set to 0 in the initial configuration
'''
base_rotation = 0.
initial_configuration = {
    'joint_mobile_base_rotation': base_rotation,
    'joint_lift': 0.6,
    'right_trigger_status': 0, # range: 0~1
    'left_trigger_status': 0, # range: 0~1
    'right_safety': 1, # range: 0~1
    'left_safety': 1, # range: 0~1
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
gripper_to_goal.update_goal(initial_configuration)

# Sleep after reaching position to ensure stability
time.sleep(5)

response_cnt = 0
interval = args.interval
while True:
    # Receive message from remote server
    message = socket.recv()
    try:
        data = json.loads(message.decode())
    except json.JSONDecodeError:
        print("Terminating the connection...")
        socket.close()
        context.term()
        break

    if response_cnt % interval == 0:
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
        
        # Get the controller rotation in quaternion form
        controller_rotation = [float(x) for x in right_controller['RightLocalRotation'].split(',')]
        
        # Get the base rotation
        base_rotation_speed = 3.14/100 # magic number: the movement would be pi/100 everytime (radian based)
        if right_thumbstick_x > 0.5:
            base_rotation -= base_rotation_speed
        elif right_thumbstick_x < -0.5:
            base_rotation += base_rotation_speed

        # Get controller position and convert to joint lift and joint arm
        _, y_str, z_str = xyz.split(',')

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
        # Send new state to update_goal
        gripper_to_goal.update_goal(mock_configuration)
    response_cnt+=1

del gripper_to_goal
