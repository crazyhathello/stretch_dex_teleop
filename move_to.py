import json
import math
import time
from typing import Tuple
import numpy as np
from stretch_body import robot as rb
from stretch_body.hello_utils import ThreadServiceExit

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def choose_closest_angle(angle1, angle2, reference_angle):
    angle1 = normalize_angle(angle1)
    angle2 = normalize_angle(angle2)
    reference_angle = normalize_angle(reference_angle)

    if abs(angle1 - reference_angle) < abs(angle2 - reference_angle):
        return angle1
    else:
        return angle2

def plan_trajectory(
    x_desired, y_desired, theta_desired, x_current, y_current, theta_current
):
    Tu = 0.5 # time constant
    # Compute the distance and angle to the desired position
    dx = x_desired - x_current
    dy = y_desired - y_current
    dtheta = normalize_angle(theta_desired - theta_current)
    dL = math.sqrt(dx ** 2 + dy ** 2)

    # Compute the time to reach the desired position
    v_max = 1  # maximum linear velocity
    omega_max = 1  # maximum angular velocity

    t_L = dL / v_max + dtheta / omega_max # time to reach the desired position
    
    N = max(int(t_L / Tu), 1) # number of steps

    if dL > 0.05:
        return (x_current + dx / N, y_current + dy / N, choose_closest_angle(np.arctan2(dy, dx), np.arctan2(-dy, -dx), theta_current))
    else:
        return (x_desired, y_desired, normalize_angle(theta_current + dtheta / N)) 


class PIDController:
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0
        self.output_limits = output_limits
        self.integral_decay = 0.99

    def update(self, error, dt):
        self.integral = self.integral * self.integral_decay + (error * dt)
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Apply output limits
        min_output, max_output = self.output_limits
        if min_output is not None:
            output = max(output, min_output)
        if max_output is not None:
            output = min(output, max_output)

        self.previous_error = error
        return output


# Initial PID parameters
Kp_v, Ki_v, Kd_v = 0.0, 0.0, 0.0
Kp_omega, Ki_omega, Kd_omega = 0.0, 0.0, 0.0
Kp_x, Ki_x, Kd_x = 2.0, 0., 0.
Kp_y, Ki_y, Kd_y = 4.0, 0.2, 0.
Kp_theta, Ki_theta, Kd_theta = 2.0, 0., 0.

min_v, max_v = -2.0, 2.0
min_omega, max_omega = -2.0, 2.0

# Define PID controllers with appropriate limits
pid_v = PIDController(Kp_v, Ki_v, Kd_v, output_limits=(min_v, max_v))
pid_omega = PIDController(
    Kp_omega, Ki_omega, Kd_omega, output_limits=(min_omega, max_omega)
)
pid_x = PIDController(Kp_x, Ki_x, Kd_x)
pid_y = PIDController(Kp_y, Ki_y, Kd_y)
pid_theta = PIDController(Kp_theta, Ki_theta, Kd_theta)

f = open("log.jsonl", "w")


def pid_policy(
    x_desired, y_desired, theta_desired, x_current, y_current, theta_current, dt
):
    global f
    # Compute errors
    e_x = x_desired - x_current
    e_y = y_desired - y_current
    e_theta = theta_desired - theta_current

    # Normalize orientation error to the range [-pi, pi]
    e_theta = (e_theta + math.pi) % (2 * math.pi) - math.pi

    # Convert position errors to robot frame
    e_x_prime = e_x * math.cos(theta_current) + e_y * math.sin(theta_current)
    e_y_prime = -e_x * math.sin(theta_current) + e_y * math.cos(theta_current)

    # Update PID controllers
    control_x = pid_x.update(e_x_prime, dt)
    control_y = pid_y.update(e_y_prime, dt)
    control_theta = pid_theta.update(e_theta, dt)

    f.write(
        json.dumps(
            {
                "x": x_current,
                "y": y_current,
                "theta": theta_current,
                "x_desired": x_desired,
                "y_desired": y_desired,
                "theta_desired": theta_desired,
                "e_x_prime": e_x_prime,
                "e_y_prime": e_y_prime,
                "e_theta": e_theta,
                "control_x": control_x,
                "control_y": control_y,
                "control_theta": control_theta,
            }
        )
        + "\n"
    )

    # Combine control actions
    v = control_x
    omega = control_y + control_theta

    # Apply limits to control signals to prevent instability
    v = max(min_v, min(v, max_v))
    omega = max(min_omega, min(omega, max_omega))

    return v, omega


# # Usage example
# x_desired, y_desired, theta_desired = 1.0, 1.0, math.pi/4
# v_desired, omega_desired = 0.5, 0.1
# x_current, y_current, theta_current = 0.0, 0.0, 0.0
# v_current, omega_current = 0.0, 0.0
# dt = 0.1

# v, omega = control_loop(x_desired, y_desired, theta_desired, v_desired, omega_desired,
#                         x_current, y_current, theta_current, v_current, omega_current, dt)


def control_loop(robot: rb.Robot):
    start_time = time.time()
    current_time = 0

    while True:
        new_current_time = time.time() - start_time
        delta_t = new_current_time - current_time
        current_time = new_current_time


        current_x, current_y, current_theta = (
            robot.base.status["x"],
            robot.base.status["y"],
            robot.base.status["theta"],
        )

        x_d, y_d, theta_d = plan_trajectory(x_desired=3, y_desired=0, theta_desired=math.pi/2,
                                           x_current=current_x, y_current=current_y, theta_current=current_theta)
        # Get the control inputs
        v, omega = pid_policy(
            x_d, y_d, theta_d, current_x, current_y, current_theta, delta_t
        )

        robot.base.set_velocity(v, omega)
        robot.push_command()

        time.sleep(1 / 80)


if __name__ == "__main__":
    robot = rb.Robot()
    robot.startup()

    try:
        # Run the control loop
        control_loop(robot)
    except (KeyboardInterrupt, SystemExit, ThreadServiceExit):
        robot.stop()

    robot.stop()
