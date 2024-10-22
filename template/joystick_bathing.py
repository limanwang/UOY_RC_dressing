import pygame
from pyrcareworld.envs.bathing_env import BathingEnv
import argparse
import time

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
    # Initialize simulation environment
    env = BathingEnv(graphics=use_graphics)
    robot = env.get_robot()
    gripper = env.get_gripper()

    # Initialize the joystick
    joystick = init_joystick()

    # Initial position
    position = [0.5, 0.5, 0.2]  # Initial position
    rotation = [0, 0, 0]        # Initial rotation angle

    mode = 1
    z = 0
    y = 0
    total_y = [0, 0, 0, 0]

    # Main control loop
    try:
        while True:
            # Get joystick input
            axis_data, button_data = get_joystick_input(joystick)
            print("axis_data, button_data: ", axis_data, button_data)

            if button_data[11]:  # Start button
                mode = 1  # Moving mode
            elif button_data[10]:  # Select button
                mode = 0  # IK mode
            
            # if mode:
                # print('controlling move base')
            # Control forward and backward movement
            if button_data[4]:  # Button Y moves forward
                robot.EnabledNativeIK(False)
                robot.MoveForward(0.5, 0.2)  # Move forward 0.5m at 0.2 speed
                env.step()
            elif button_data[0]:  # Button A moves backward
                robot.EnabledNativeIK(False)
                robot.MoveBack(0.5, 0.2)  # Move backward 0.5m at 0.2 speed
                env.step()

            # Control left and right turns
            if button_data[3]:  # Button X turns left
                robot.EnabledNativeIK(False)
                robot.TurnLeft(90, 0.2)  # Turn left 90 degrees at 0.2 speed
                env.step()
            elif button_data[1]:  # Button B turns right
                robot.EnabledNativeIK(False)
                robot.TurnRight(90, 0.2)  # Turn right 90 degrees at 0.2 speed
                env.step()

            if button_data[9]:  # Button R2 (brake)
                robot.MoveForward(0.5, 0)  # Move forward 0.5m at 0 speed
                env.step()

            # else:
            #     print('controlling arm')
            # Use IK to precisely control position
            move_z = axis_data[1] * 0.1  # Left joystick controls forward/backward
            move_y = axis_data[3] * 0.1  # Right joystick controls up/down
            y += move_y

            if y <= -10:
                y = -10
            elif y >= 0:
                y = 0

            quotient, remainder = divmod(y, -2)
            if quotient == 1:
                total_y = [-2, remainder, 0, 0]
            elif quotient == 2:
                total_y = [-2, -2, remainder, 0]
            elif quotient == 3:
                total_y = [-2, -2, -2, remainder]
            elif quotient == 0:
                total_y = [remainder, 0, 0, 0]

            z += move_z
            if z >= 2:
                z = 2
            elif z <= -10:
                z = -10
            
            print('z is:', z)
            print('y is:', y)
            print('y is:', total_y)

            robot.EnabledNativeIK(False)
            robot.SetJointVelocity([0, 0, z, total_y[0], total_y[1], total_y[2], total_y[3], 0, 0, 0, 0, 0, 0, 0])
            env.step()

            # Control gripper open and close
            if button_data[6]:  # Button L1 opens the gripper
                gripper.GripperOpen()
                env.step()
            elif button_data[7]:  # Button R1 closes the gripper
                gripper.GripperClose()
                env.step()

            # Execute environment step
            env.step(1)

            # Check for exit condition
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            # Delay to reduce CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Control ended")

    finally:
        # Clean up and exit
        pygame.quit()
        env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation with joystick control.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)

