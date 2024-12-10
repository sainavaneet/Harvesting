
import numpy as np

GRIPPER_JOINT_OPEN = 1.4910
GRIPPER_JOINT_CLOSE = 0.4



TARGETS = {
    'cucu1': {'position': {'x': 0.35, 'y': 0.055, 'z': 0.54}, 'initial_pose': {'x': 0.2, 'y': 0, 'z': 0.6}},
    'cucu2': {'position': {'x': 0.32, 'y': -0.01, 'z': 0.7}, 'initial_pose': {'x': 0.2, 'y': 0, 'z': 0.67}},
    'cucu3': {'position': {'x': 0.34, 'y': 0, 'z': 0.37}, 'initial_pose': {'x': 0.15, 'y': 0, 'z': 0.35}},
    'cucu4': {'position': {'x': 0.32, 'y': 0, 'z': 0.7}, 'initial_pose': {'x': 0.15, 'y': 0, 'z': 0.6}},
    'cucu5': {'position': {'x': 0.37, 'y': -0.03, 'z': 0.37}, 'initial_pose': {'x': 0.15, 'y': 0, 'z': 0.35}},
    'cucu6': {'position': {'x': 0.32, 'y': -0.02, 'z': 0.7}, 'initial_pose': {'x': 0.2, 'y': 0, 'z': 0.6}},
    'cucu7': {'position': {'x': 0.37, 'y': -0.03, 'z': 0.37}, 'initial_pose': {'x': 0.15, 'y': 0, 'z': 0.35}},
    'cucu8': {'position': {'x': 0.34, 'y': -0.03, 'z': 0.55}, 'initial_pose': {'x': 0.2, 'y': 0, 'z': 0.5}},
    'cucu9': {'position': {'x': 0.37, 'y': -0.04, 'z': 0.28}, 'initial_pose': {'x': 0.2, 'y': 0, 'z': 0.3}},
}


MODEL_PATH = "/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train2/weights/best.pt"


COLOUR_IMAGE_TOPIC = '/camera/color/image_raw'

DEPTH_IMAGE_TOPIC = '/camera/depth/image_rect_raw'


INTRINSIC = np.array([[605.9666137695312, 0.0, 322.66650390625],
                              [ 0.0, 605.8338623046875, 248.94570922851562],
                              [ 0.0, 0.0, 1.0]])

THR = 100000