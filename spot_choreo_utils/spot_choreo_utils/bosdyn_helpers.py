# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

from bosdyn.api.robot_state_pb2 import RobotState
from bosdyn.client import (
    frame_helpers,
)
from bosdyn.client.math_helpers import SE3Pose
from spatialmath import (
    SE3,
    UnitQuaternion,
)
from spot_wrapper.wrapper import SpotWrapper

ROBOT_STATE_TO_ANIMATION_NAME = {
    "arm.sh0": "shoulder_0",
    "arm.sh1": "shoulder_1",
    "arm.el0": "elbow_0",
    "arm.el1": "elbow_1",
    "arm.wr0": "wrist_0",
    "arm.wr1": "wrist_1",
    "arm.f1x": "gripper",
    "body_pos_x": "body_pos_x",
    "body_pos_y": "body_pos_y",
    "body_pos_z": "body_pos_z",
    "body_quat_w": "body_quat_w",
    "body_quat_x": "body_quat_x",
    "body_quat_y": "body_quat_y",
    "body_quat_z": "body_quat_z",
    "fl.hx": "front_left_hip_x",
    "fl.hy": "front_left_hip_y",
    "fl.kn": "front_left_knee",
    "fr.hx": "front_right_hip_x",
    "fr.hy": "front_right_hip_y",
    "fr.kn": "front_right_knee",
    "hl.hx": "rear_left_hip_x",
    "hl.hy": "rear_left_hip_y",
    "hl.kn": "rear_left_knee",
    "hr.hx": "rear_right_hip_x",
    "hr.hy": "rear_right_hip_y",
    "hr.kn": "rear_right_knee",
}


def SE3PoseToSE3(bosdyn_pose: SE3Pose) -> SE3:
    """Converts a bosdyn SE3Pose to spatialmath SE3"""
    quat = bosdyn_pose.rotation
    converted_rot = UnitQuaternion([quat.w, quat.x, quat.y, quat.z])
    return SE3.Rt(t=bosdyn_pose.get_translation(), R=converted_rot.SO3())


def get_world_t_body(robot_state: RobotState) -> SE3:
    """Extract the current transform between world origin and spot's body

    Spot's world 0 is wherever the robot powered on. Choreography 0 is wherever the animation started,
    so measuring, storing, and subtracting world_t_body is essential to building animation

    Returns:
        SE3: The spatialmath body pose of the robot.
    """
    vision_tform_body = frame_helpers.get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)
    return SE3PoseToSE3(vision_tform_body)


def get_current_joint_pos(robot: SpotWrapper) -> dict[str, float]:
    """Returns a dictionary of current joint poses and body pose in world frame"""
    full_state = robot._robot_state_client.get_robot_state()
    joint_states = full_state.kinematic_state.joint_states
    joint_dict = {
        map_robot_state_to_animation_name(joint_state.name): joint_state.position.value for joint_state in joint_states
    }

    # Build the body pose
    body_state = get_world_t_body(full_state)
    body_t = body_state.t
    body_quat = body_state.UnitQuaternion().vec
    body_pose_names = [f"body_pos_{dim}" for dim in ["x", "y", "z"]]
    body_rot_names = [f"body_quat_{dim}" for dim in ["w", "x", "y", "z"]]
    joint_dict.update(dict(zip(body_pose_names, body_t)))
    joint_dict.update(dict(zip(body_rot_names, body_quat)))
    return joint_dict


def map_robot_state_to_animation_name(robot_state_name: str) -> str:
    """Maps the joint names from bosdyn's robot_state message to animation names"""
    if robot_state_name not in ROBOT_STATE_TO_ANIMATION_NAME.keys():
        return ""
    return ROBOT_STATE_TO_ANIMATION_NAME[robot_state_name]
