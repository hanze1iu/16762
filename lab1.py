import time
import numpy as np
import stretch_body.robot

robot = stretch_body.robot.Robot()
robot.startup()
robot.stow()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.arm.move_to(0.5)  # Extend the telescoping arm all the way out
robot.lift.move_to(1.1) # Raise the lift all the way up
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_yaw', np.radians(5))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_pitch', np.radians(5))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_roll', np.radians(15))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper', 100)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper', -50)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.head.move_by('head_pan', np.radians(45))  # Move head pan
robot.head.move_by('head_tilt', np.radians(45)) # Move head tilt
robot.push_command() 
robot.head.wait_until_at_setpoint()
    

robot.stow()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.base.translate_by(0.5)  # Move 0.5 forward
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.base.rotate_by(np.radians(180))
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.base.translate_by(0.5)
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.stop()