import time
import numpy as np
import stretch_body.robot
robot = stretch_body.Robot()
robot.starup()
robot.stow()

robot.arm.move_to(0.5)  # Extend the telescoping arm all the way out
robot.liftft.move_to(1.1) #raise the lift all the way up
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_yaw', np.radians(30))
robot.end_of_arm.wait_until_at_setpoint()
robot.end_of_arm.move_to('wrist_pitch', np.radians(30))
robot.end_of_arm.wait_until_at_setpoint()
robot.end_of_arm.move_to('wrist_roll', np.radians(30))
robot.end_of_arm.wait_until_at_setpoint()
robot.end_of_arm.move_to('stretch_gripper', 50)

