import pygame
from pyrcareworld.envs.dressing_env import DressingEnv
import numpy as np
import cv2
# from ultralytics import YOLO
import argparse

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Initialized Joystick: {joystick.get_name()}")
    return joystick

def get_joystick_input(joystick):
    pygame.event.pump()
    axis_data = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    button_data = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    return axis_data, button_data

def _main(use_graphics=False):
    # Initialize environment and model
    env = DressingEnv(graphics=use_graphics)
    # model = YOLO("yolov8n-pose.pt")
    kinova_id = 315893
    robot = env.GetAttr(kinova_id)
    gripper = env.GetAttr(3158930)
    camera_hand = env.GetAttr(654321)

    # Initialize the joystick
    joystick = init_joystick()

    # Initial position and rotation
    position = [1.5, 2.3, 0.6]
    rotation = [0, 0, -90]

    # Main control loop
    try:
        while True:
            # Get joystick input
            axis_data, button_data = get_joystick_input(joystick)
            print('axis_data, button_data:', axis_data, button_data)

            # Map joystick axis data to robot position control
            move_x = axis_data[0] * -0.02  # Move left or right
            move_y = axis_data[1] * -0.02  # Move up or down
            # move_z = axis_data[3] * 0.1  
            move_z = 0
            if button_data[0]:  # Button A moves forward (towards the screen)
                move_z = 0.02
            elif button_data[4]:  # Button Y moves backward (into the screen)
                move_z = -0.02

            # Update robot position
            position[0] += move_x
            position[1] += move_y
            position[2] += move_z

            # Control gripper open and close
            if button_data[3]:  # Button X opens the gripper
                gripper.GripperOpen()
                env.step()
            elif button_data[1]:  # Button B closes the gripper
                gripper.GripperClose()
                env.step()

            # Control rotation
            rotate_x = axis_data[2] * 5  # Control roll (self-rotation)
            rotate_z = axis_data[3] * -5  # Control pitch (gripper up and down)
            rotate_y = 0
            if button_data[6]:  # Button L1 rotates left
                rotate_y = 5
            elif button_data[7]:  # Button R1 rotates right
                rotate_y = -5

            rotation[0] += rotate_x
            rotation[1] += rotate_y
            rotation[2] += rotate_z

            if axis_data[5] > 0.5:
                robot.IKTargetDoKill()
                env.step()
            else:
                # Control robot movement
                robot.IKTargetDoMove(position=position, duration=2, speed_based=False)
                env.step()
                robot.IKTargetDoRotate(rotation=rotation, duration=2, speed_based=False)
                env.step()

            # Execute environment step
            env.step(2)

            # Check for exit conditions
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("Control ended")

    finally:
        # Clean up and exit
        pygame.quit()
        env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld dressing environment simulation with joystick control.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)

