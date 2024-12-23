# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

from collections import namedtuple
from typing import Dict

from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    AnimationKeyframe,
)
from spatialmath import (
    SE3,
    SO3,
    UnitQuaternion,
)
from spot_wrapper.wrapper import SpotWrapper

from spot_choreo_utils.bosdyn_helpers import get_current_joint_pos, get_world_t_body
from spot_choreo_utils.choreo_creation.choreo_builders.spot_properties import get_joint_angle_names

SE3AsDict = namedtuple("SE3AsDict", "x y z qw qx qy qz")


def extract_body_angles(keyframe: AnimationKeyframe) -> Dict[str, float]:
    """Extract the body angles from the animation proto into a flat dictionary of joint to angle"""
    body_angle_names = get_joint_angle_names()["body"]

    body_pos = keyframe.body.body_pos
    body_rot = keyframe.body.quaternion

    body_angles = [getattr(body_pos, attr).value for attr in body_pos.DESCRIPTOR.fields_by_name.keys()]
    body_angles += [getattr(body_rot, attr) for attr in body_rot.DESCRIPTOR.fields_by_name.keys()]
    return dict(zip(body_angle_names, body_angles, strict=True))


def extract_gripper_angles(keyframe: AnimationKeyframe) -> Dict[str, float]:
    """Extract the gripper angle from the animation proto into a flat dictionary of joint to angle"""
    values = [keyframe.gripper.gripper_angle.value]
    key_names = get_joint_angle_names()["gripper"]
    keyframe_dictionary = dict(zip(key_names, values, strict=True))
    return keyframe_dictionary


def extract_leg_angles(keyframe: AnimationKeyframe) -> Dict[str, float]:
    """Extract the leg angles from the animation proto into a flat dictionary of joint to angle"""

    joint_angle_names = get_joint_angle_names()
    leg_keys = filter(lambda x: "leg" in x, joint_angle_names)
    leg_name_map = dict(zip(keyframe.legs.DESCRIPTOR.fields_by_name.keys(), leg_keys, strict=True))

    keyframe_dictionary = {}
    # Extract each leg's joint angles
    for proto_key, leg_name in leg_name_map.items():
        joint_containers = getattr(keyframe.legs, proto_key).joint_angles
        leg_angles = [getattr(joint_containers, key) for key in joint_containers.DESCRIPTOR.fields_by_name.keys()]
        specific_leg_keys = joint_angle_names[leg_name]
        keyframe_dictionary.update(dict(zip(specific_leg_keys, leg_angles, strict=True)))

    return keyframe_dictionary


def extract_arm_angles(keyframe: AnimationKeyframe) -> Dict[str, float]:
    """Extract the arm angles from the animation proto into a flat dictionary of joint to angle"""

    arm_angles = keyframe.arm.joint_angles
    angle_keys = arm_angles.DESCRIPTOR.fields_by_name.keys()
    arm_values = [getattr(arm_angles, key).value for key in angle_keys]
    keyframe_dictionary = dict(zip(angle_keys, arm_values, strict=True))
    return keyframe_dictionary


def get_joint_angles_at_keyframe(animation: Animation, keyframe_idx: int) -> Dict[str, float]:
    """Extract all joint angles for a specific keyframe idx"""

    if keyframe_idx > len(animation.animation_keyframes) or keyframe_idx < 0:
        print(f"Animation of length {len(animation.animation_keyframes)} doesn't have keyframe {keyframe_idx}")
        return {}
    keyframe = animation.animation_keyframes[keyframe_idx]

    keyframe_keys = {}
    keyframe_keys.update(extract_gripper_angles(keyframe))
    keyframe_keys.update(extract_arm_angles(keyframe))
    keyframe_keys.update(extract_leg_angles(keyframe))
    keyframe_keys.update(extract_body_angles(keyframe))
    return keyframe_keys


def se3_to_se3_vec(se3: SE3) -> SE3AsDict:
    """Extract SE3 proto into a named tuple"""
    q = UnitQuaternion(se3)
    return SE3AsDict(x=se3.x, y=se3.y, z=se3.z, qw=q.vec_xyzs[3], qx=q.vec_xyzs[0], qy=q.vec_xyzs[1], qz=q.vec_xyzs[2])


def update_body_to_animation_frame(
    joint_angles: Dict[str, float], robot: SpotWrapper, body_zero_frame_in_world_coordinates: SE3
) -> None:
    """
    Overwrite the body angles with values measured from the robot relative to the passed in body zero frame.
    This function is a bridge between choreography bugs and Bosdyn APIs.
       1) There is no way to directly measure body position in the body frame
       2) The choreography record animation function doesn't properly capture body quaternions
    So by manually saving a body_zero frame and then calculating the relative body rotation this function
    calculates a measured body position that can be sent to the robot.
    """

    body_angle_names = get_joint_angle_names()["body"]

    # Take the difference between the currently measured pose and the stored
    measured_pose = get_world_t_body(robot._robot_state_client.get_robot_state())
    body_frame_change_from_zero_frame = SE3.Rt(
        t=measured_pose.t - body_zero_frame_in_world_coordinates.t,
        R=SO3(body_zero_frame_in_world_coordinates.R).inv() * measured_pose.R,
    )

    # Transform the proto into a named tuple that can be zipped together with values
    pose_as_dict = se3_to_se3_vec(body_frame_change_from_zero_frame)
    print(pose_as_dict)

    angles_measured_directly = dict(zip(body_angle_names, pose_as_dict, strict=True))
    for key in angles_measured_directly:
        joint_angles[key] = angles_measured_directly[key]


def get_pose_in_animation_frame(robot: SpotWrapper, body_zero_frame_in_world_coordinates: SE3) -> Dict[str, float]:
    """
    The animation frame is a concept that doesn't exist outside of choreography.
    All robot motions are relative to the feet's position when the animation starts.
    Therefore, when calculating the joint angles we need an offset between the
    vision frame and animation frame base. Grab the animation frame base at the
    location the animation starts and hold onto it for all future keyframes.
    When jumping backwards and forwards in an animation this process may become more complicated.
    """
    # Record a tiny animation to extract all joint poses
    joint_pos = get_current_joint_pos(robot)
    update_body_to_animation_frame(joint_pos, robot, body_zero_frame_in_world_coordinates)
    return joint_pos
