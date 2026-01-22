import time
import numpy as np
import stretch_body.robot
robot = stretch_body.Robot()
robot.startup()
robot.stow()

robot.arm.move_to(0.5)  # Extend the telescoping arm all the way out
robot.lift.move_to(1.1) #raise the lift all the way up
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_yaw', np.radians(30))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_pitch', np.radians(30))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_roll', np.radians(30))
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper', 100)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.head.move_by('head_pan', np.radians(45)) # Move head pan
robot.head.move_by('head_tilt', np.radians(45)) # Move head tilt
robot.push_command() 
time.sleep(2.0)

robot.stow()

robot.base.translate_by(0.5) # move 0.5 foward
robot.push_command()
time.sleep(3.0)
robot.base.rotate_by(np.radians(180))
robot.push_command()
time.sleep(3.0)
robot.base.translate_by(0.5)
robot.push_command()
time.sleep(3.0)

robot.stop()