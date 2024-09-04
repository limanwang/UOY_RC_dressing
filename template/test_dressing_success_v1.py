from pyrcareworld.envs.dressing_env import DressingEnv
import pyrcareworld.attributes as attr
import cv2
import os
import numpy as np
import json
import argparse
from ultralytics import YOLO

def _main(use_graphics=False):
    if use_graphics:
        text = """
        An example of the usage of the dressing environment.

        The robot will move to the first position, pick up the cloth, and move to the second position to drop the cloth.

        You can obtain low level information of the cloth, the robot, and use unlimited numbers of cameras to observe the scene.

        Check the website detailed rubric. After each run of the simulation, a json file will be generated in the current directory
        (~/.config/unity3d/RCareWorld/DressingPlayer).
        The path may be different accotding to the OS and your computer configuration.
        """
            
        print(text)
    # Initialize the environment
    # env = DressingEnv(graphics=use_graphics)
    # Load a model
    # model = YOLO("yolov8x-pose-p6.pt")  # load an official model
    model = YOLO("yolov8n-pose.pt")  # load an official model

    env = DressingEnv(graphics=use_graphics)
    print(env.attrs)

    kinova_id = 315893
    robot = env.GetAttr(kinova_id)
    env.step()

    # Get the gripper attribute and open the gripper
    gripper = env.GetAttr(3158930)
    gripper.GripperOpen()
    env.step(300)

    # gripper.GripperClose()
    # env.step(300)
    

    # Get the cloth attribute and perform a simulation step
    cloth = env.GetAttr(782563)
    env.step()
    # print(cloth.data)

    # Set the camera as a child of the robot's hand
    # you can also load new cameras and set them as children of other objects in the scene
    camera_hand = env.GetAttr(654321)
    camera_hand.SetTransform(position=gripper.data['position'], rotation=[-90, 90, -90])
    camera_hand.SetParent(3158930)


    # Random positions and rotation
    # 横向，z，y轴
    position1 = (1.5, 2.3, 0.6)
    rotation2 = (0, 0, -90)

    env.step(300)

    # # Move the robot to the first position
    # robot.IKTargetDoMove(
    #     position=[position1[0], position1[1], position1[2]],
    #     duration=2,
    #     speed_based=False,
    # )
    # robot.WaitDo()

    # Move the robot to the position
    # robot.IKTargetDoMove(
    #     position=[position1[0]+0.42, position1[1]-0.55, position1[2]-0.7],
    #     duration=2,
    #     speed_based=False,
    # )
    # robot.WaitDo()
 
     # Move the robot to the first position
    robot.IKTargetDoMove(
        position=[position1[0]+0.4, position1[1]-0.3, position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    robot.IKTargetDoRotate(
    rotation=[rotation2[0], rotation2[1], rotation2[2]],
    duration=2,
    speed_based=False,
    )
    robot.WaitDo()

    robot.IKTargetDoRotate(
    rotation=[rotation2[0], rotation2[1]+135, rotation2[2]],
    duration=2,
    speed_based=False,
    )
    robot.WaitDo()


    # Move the robot to the grasp position
    # position1[0]+0.53
    # position1[1]-0.15
    # position1[2]-0.63
    robot.IKTargetDoMove(
        position=[position1[0]+0.46, position1[1]-0.13, position1[2]-0.51],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()


    # robot.IKTargetDoRotate(
    #     rotation=[rotation[0], rotation[1], rotation[2]],
    #     duration=2,
    #     speed_based=False,
    # )
    # robot.WaitDo()

    # robot.IKTargetDoRotate(
    # rotation=[rotation2[0], rotation2[1], rotation2[2]],
    # duration=2,
    # speed_based=False,
    # )
    # robot.WaitDo()

    # Retrieve cloth particle data after moving the robot
    cloth.GetParticles()
    env.step()


    gripper.GripperClose()
    env.step(300)
    # env.step(300)

    cloth.AddAttach(id=3158930, max_dis=0.4)

    # robot.set_cloth_and_robot(cloth_id=3158930,cloth_name='cloth',robot_id=315893,gripper_name='gripper',grasp_radius=0.5)

    # Move the robot to the grasping position
    robot.IKTargetDoMove(
        position=[position1[0]+0.1, position1[1]-0.1, position1[2]-0.1],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()


        # Move the robot to the position
    # robot.IKTargetDoMove(
    #     position=[position1[0]+0.45, position1[1]-0.55, position1[2]],
    #     duration=2,
    #     speed_based=False,
    # )
    # robot.WaitDo()  

        # Move the robot to the dressing position
    # robot.IKTargetDoMove(
    #     position=[position1[0]+0.3, position1[1]-0.6, position1[2]-0.6],
    #     duration=2,
    #     speed_based=False,
    # )
    # robot.WaitDo()

    # 旋转cloth 180
    robot.IKTargetDoRotate(
    rotation=[rotation2[0], rotation2[1], rotation2[2]-90],
    duration=2,
    speed_based=False,
    )
    robot.WaitDo()

    robot.IKTargetDoRotate(
    rotation=[rotation2[0], rotation2[1]+90, rotation2[2]-90],
    duration=2,
    speed_based=False,
    )
    robot.WaitDo()


    # 走去观测点
     # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.3, position1[1]-0.5, position1[2]+0.2],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    robot.IKTargetDoRotate(
    # (roll, pitch, yaw)
    rotation=[rotation2[0]+98, rotation2[1]+90, rotation2[2]],
    duration=2,
    speed_based=False,
    )
    robot.WaitDo()

    camera_hand.GetRGB(512, 512,fov = 120.0)
    env.step()
    rgb = np.frombuffer(camera_hand.data["rgb"], dtype=np.uint8)
    env.step()
    rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
    cv2.imwrite("rgb_hand3.png", rgb)

    env.step(300)
    results = model(["rgb_hand3.png"])

    # Initialize a list to store the x-coordinates of the left wrist points
    left_wrist_x_coords = []

    for result in results:
        boxes = result.boxes  # Boxes object for bounding box outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs
        result.show()  # display to screen
        result.save(filename="rgb_hand3_pose.jpg")  # save to disk

        if keypoints is not None:
            # Accessing the left wrist keypoint (index 9) for each detected person
            for i, person_keypoints in enumerate(keypoints.data):
                left_wrist_point = person_keypoints[9][:2]  # Get x, y coordinates (ignore confidence)
                left_wrist_x_coords.append(left_wrist_point[0].item())  # Append the x-coordinate to the list
                confidence = person_keypoints[9][2]  # Get the confidence score
                print(f"Person {i + 1} - Left wrist point (index 9): {left_wrist_point}, Confidence: {confidence.item()}")

    # Calculate the mean of the x-coordinates
    if left_wrist_x_coords:
        mean_x = sum(left_wrist_x_coords) / len(left_wrist_x_coords)
        print(f"Mean of the x-coordinates of all left wrist points: {mean_x}")

    # Calculate deta based on mean_x
    if mean_x != 312:
        deta = -(mean_x - 312) * 0.002
        print(f"Calculated deta: {deta}")
    else:
        deta = 0
        print("mean_x is equal to 312, deta is set to 0")

    env.step(300)



    # robot.IKTargetDoRotate(
    # rotation=[rotation2[0], rotation2[1]+180, rotation2[2]-90],
    # duration=2,
    # speed_based=False,
    # )
    # robot.WaitDo()



    robot.IKTargetDoMove(
        position=[position1[0]-0.56 + deta, position1[1]-0.6, position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the dressing position
    robot.IKTargetDoMove(
        position=[position1[0]-0.54 + deta, position1[1]-0.65, position1[2]-0.3],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the dressing position
    robot.IKTargetDoMove(
        position=[position1[0]-0.53 + deta, position1[1]-0.65, position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.52 + deta, position1[1]-0.65, position1[2]-0.1],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.52 + deta, position1[1]-0.65, position1[2]-0.2],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.51 + deta, position1[1]-0.65, position1[2]-0.3],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.51 + deta, position1[1]-0.65, position1[2]-0.4],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

        # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.51 + deta, position1[1]-0.65, position1[2]-0.5],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

            # Move the robot to the position
    robot.IKTargetDoMove(
        position=[position1[0]-0.51 + deta, position1[1]-0.65, position1[2]-0.55],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()




    gripper.GripperOpen()
    env.step(300)
    cloth.RemoveAttach(3158930)



    env.step(30000)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld dressing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
