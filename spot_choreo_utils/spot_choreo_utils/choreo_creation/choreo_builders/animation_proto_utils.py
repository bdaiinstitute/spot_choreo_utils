# Copyright (c) 2024-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import math
from typing import Any, Dict, Optional

from bosdyn.api.geometry_pb2 import Quaternion, Vec3Value
from bosdyn.api.spot.choreography_sequence_pb2 import (
    AnimateArm,
    AnimateBody,
    AnimateGripper,
    AnimateLegs,
    AnimateSingleLeg,
    AnimationKeyframe,
    ArmJointAngles,
    LegJointAngles,
)
from google.protobuf.wrappers_pb2 import DoubleValue


def protobuf_zero_omission_check(value: float) -> bool:
    """
    Returns whether the value would be omitted by protobuf when sent over the wire
    """
    if value == 0.0 and math.copysign(1, value) == 1.0:
        return True
    return False


def ensure_protobuf_compliance(value: Optional[float], adjust_positive: bool = True) -> Optional[float]:
    """
    Protobufs don't include zero as a value to reduce wire traffic, but choreography service
    will complain if data is included sometimes and not others.
    Update values set to zero to an almost-zero number so they are sent as
    part of the proto
    """
    if value is not None and protobuf_zero_omission_check(value):
        return 1e-6 if adjust_positive else -1e-6
    return value


def check_if_protobuf_field_set(proto: Any, field: str) -> bool:
    """
    Protobuf pretends that all fields exist when accessed directly, so you can't check
    for presence with direct access. But the choreography service on robot cares.
    This function provides a workaround to check whether the field is actually set
    """
    all_field_pairs = proto.ListFields()
    for descriptor, _ in all_field_pairs:
        if field == descriptor.name:
            return True
    return False


def check_if_keyframe_poses_equivalent(keyframe_one: AnimationKeyframe, keyframe_two: AnimationKeyframe) -> bool:
    """
    Checks to see if animation keyframes specify the same robot pose. Time of keyframe is ignored.
    """

    grippers_match = keyframe_one.gripper == keyframe_two.gripper
    arms_match = keyframe_one.arm == keyframe_two.arm
    bodys_match = keyframe_one.body == keyframe_two.body
    legs_match = keyframe_one.legs == keyframe_two.legs

    return grippers_match and arms_match and bodys_match and legs_match


def protobuf_joint_angle_map() -> Dict[str, list[str]]:
    """Returns a map of which joint angles are part of each protobuf paramaterization"""

    leg_joint_suffixes = ["_hip_x", "_hip_y", "_knee"]
    return {
        "gripper": ["gripper"],
        "arm": ["shoulder_0", "shoulder_1", "elbow_0", "elbow_1", "wrist_0", "wrist_1"],
        "body": [
            "body_pos_x",
            "body_pos_y",
            "body_pos_z",
            "body_quat_w",
            "body_quat_x",
            "body_quat_y",
            "body_quat_z",
        ],
        "front_left_leg": [f"front_left{suffix}" for suffix in leg_joint_suffixes],
        "front_right_leg": [f"front_right{suffix}" for suffix in leg_joint_suffixes],
        "rear_left_leg": [f"rear_left{suffix}" for suffix in leg_joint_suffixes],
        "rear_right_leg": [f"rear_right{suffix}" for suffix in leg_joint_suffixes],
    }


def build_gripper_params(gripper: Optional[float] = None) -> AnimateGripper:
    """Create the Gripper's animation paramater protobuf"""

    gripper_animation = AnimateGripper()
    if gripper is not None:
        gripper_animation.gripper_angle.CopyFrom(DoubleValue(value=gripper))
    return gripper_animation


def build_arm_params(
    elbow_0: Optional[float] = None,
    elbow_1: Optional[float] = None,
    shoulder_0: Optional[float] = None,
    shoulder_1: Optional[float] = None,
    wrist_0: Optional[float] = None,
    wrist_1: Optional[float] = None,
) -> ArmJointAngles:
    """Create the Arm's animation paramater protobuf"""

    # Protos and the animation service don't play nicely with 0
    elbow_0 = ensure_protobuf_compliance(elbow_0)
    elbow_1 = ensure_protobuf_compliance(elbow_1)
    shoulder_0 = ensure_protobuf_compliance(shoulder_0)
    shoulder_1 = ensure_protobuf_compliance(shoulder_1)
    wrist_0 = ensure_protobuf_compliance(wrist_0)
    wrist_1 = ensure_protobuf_compliance(wrist_1)

    arm_joint_angles = ArmJointAngles()

    if elbow_0:
        arm_joint_angles.elbow_0.CopyFrom(DoubleValue(value=elbow_0))
    if elbow_1:
        arm_joint_angles.elbow_1.CopyFrom(DoubleValue(value=elbow_1))

    if shoulder_0:
        arm_joint_angles.shoulder_0.CopyFrom(DoubleValue(value=shoulder_0))
    if shoulder_1:
        arm_joint_angles.shoulder_1.CopyFrom(DoubleValue(value=shoulder_1))

    if wrist_0:
        arm_joint_angles.wrist_0.CopyFrom(DoubleValue(value=wrist_0))
    if wrist_1:
        arm_joint_angles.wrist_1.CopyFrom(DoubleValue(value=wrist_1))

    arm_animation = AnimateArm()
    arm_animation.joint_angles.CopyFrom(arm_joint_angles)

    return arm_animation


def build_body_params(
    body_pos_x: Optional[float] = None,
    body_pos_y: Optional[float] = None,
    body_pos_z: Optional[float] = None,
    body_quat_x: Optional[float] = None,
    body_quat_y: Optional[float] = None,
    body_quat_z: Optional[float] = None,
    body_quat_w: Optional[float] = None,
) -> AnimateBody:
    """Create the Body's animation paramater protobuf"""

    # Protos and the animation service don't play nicely with 0
    body_pos_x = ensure_protobuf_compliance(body_pos_x)
    body_pos_y = ensure_protobuf_compliance(body_pos_y)
    body_pos_z = ensure_protobuf_compliance(body_pos_z)
    body_quat_x = ensure_protobuf_compliance(body_quat_x)
    body_quat_y = ensure_protobuf_compliance(body_quat_y)
    body_quat_z = ensure_protobuf_compliance(body_quat_z)
    body_quat_w = ensure_protobuf_compliance(body_quat_w)

    position = None
    quaternion = None

    if body_pos_z and body_quat_w:
        position = Vec3Value(z=DoubleValue(value=body_pos_z))
        quaternion = Quaternion(w=body_quat_w)

    if body_pos_x and body_pos_y and body_pos_z:
        position = Vec3Value(
            x=DoubleValue(value=body_pos_x), y=DoubleValue(value=body_pos_y), z=DoubleValue(value=body_pos_z)
        )
    if body_quat_x and body_quat_y and body_quat_z and body_quat_w:
        quaternion = Quaternion(
            x=body_quat_x,
            y=body_quat_y,
            z=body_quat_z,
            w=body_quat_w,
        )

    body_animation = AnimateBody()
    if position:
        body_animation.body_pos.CopyFrom(position)
    if quaternion:
        body_animation.quaternion.CopyFrom(quaternion)

    return body_animation


def build_leg_params(
    front_left_hip_x: Optional[float] = None,
    front_left_hip_y: Optional[float] = None,
    front_left_knee: Optional[float] = None,
    front_right_hip_x: Optional[float] = None,
    front_right_hip_y: Optional[float] = None,
    front_right_knee: Optional[float] = None,
    rear_left_hip_x: Optional[float] = None,
    rear_left_hip_y: Optional[float] = None,
    rear_left_knee: Optional[float] = None,
    rear_right_hip_x: Optional[float] = None,
    rear_right_hip_y: Optional[float] = None,
    rear_right_knee: Optional[float] = None,
) -> AnimateLegs:
    """Create the Legs' animation paramater protobuf"""

    # Protos and the animation service don't play nicely with 0
    front_left_hip_x = ensure_protobuf_compliance(front_left_hip_x)
    front_left_hip_y = ensure_protobuf_compliance(front_left_hip_y)
    front_left_knee = ensure_protobuf_compliance(front_left_knee)
    front_right_hip_x = ensure_protobuf_compliance(front_right_hip_x)
    front_right_hip_y = ensure_protobuf_compliance(front_right_hip_y)
    front_right_knee = ensure_protobuf_compliance(front_right_knee)
    rear_left_hip_x = ensure_protobuf_compliance(rear_left_hip_x)
    rear_left_hip_y = ensure_protobuf_compliance(rear_left_hip_y)
    rear_left_knee = ensure_protobuf_compliance(rear_left_knee)
    rear_right_hip_x = ensure_protobuf_compliance(rear_right_hip_x)
    rear_right_hip_y = ensure_protobuf_compliance(rear_right_hip_y)
    rear_right_knee = ensure_protobuf_compliance(rear_right_knee)

    front_left_angles = None
    front_right_angles = None
    rear_left_angles = None
    rear_right_angles = None

    if front_left_hip_x and front_left_hip_y and front_left_knee:
        front_left_angles = LegJointAngles(hip_x=front_left_hip_x, hip_y=front_left_hip_y, knee=front_left_knee)
    if front_right_hip_x and front_right_hip_y and front_left_knee:
        front_right_angles = LegJointAngles(hip_x=front_right_hip_x, hip_y=front_right_hip_y, knee=front_left_knee)
    if rear_left_hip_x and rear_left_hip_y and rear_left_knee:
        rear_left_angles = LegJointAngles(hip_x=rear_left_hip_x, hip_y=rear_left_hip_y, knee=rear_left_knee)
    if rear_right_hip_x and rear_right_hip_y and rear_right_knee:
        rear_right_angles = LegJointAngles(hip_x=rear_right_hip_x, hip_y=rear_right_hip_y, knee=rear_right_knee)

    front_left_leg = None
    front_right_leg = None
    rear_left_leg = None
    rear_right_leg = None

    if front_left_angles:
        front_left_leg = AnimateSingleLeg()
        front_left_leg.joint_angles.CopyFrom(front_left_angles)
    if front_right_angles:
        front_right_leg = AnimateSingleLeg()
        front_right_leg.joint_angles.CopyFrom(front_right_angles)
    if rear_left_angles:
        rear_left_leg = AnimateSingleLeg()
        rear_left_leg.joint_angles.CopyFrom(rear_left_angles)
    if rear_right_angles:
        rear_right_leg = AnimateSingleLeg()
        rear_right_leg.joint_angles.CopyFrom(rear_right_angles)

    animate_legs = AnimateLegs()
    if front_left_leg:
        animate_legs.fl.CopyFrom(front_left_leg)
    if front_right_leg:
        animate_legs.fr.CopyFrom(front_right_leg)
    if rear_left_leg:
        animate_legs.hl.CopyFrom(rear_left_leg)
    if rear_right_leg:
        animate_legs.hr.CopyFrom(rear_right_leg)

    return animate_legs


def joint_angle_keyframe_to_proto(
    keyframe_angles: Dict[str, float], start_time: Optional[float] = None
) -> Optional[AnimationKeyframe]:
    """
    Converts a dictionary of joint angles into an animation keyframe
    """
    joint_angle_names = protobuf_joint_angle_map()

    def extract_paramaters(body_section: str) -> Optional[Dict[str, float]]:
        """Returns a dictionary of joint angle names and angles for a given body section"""
        params = None

        if any(k in keyframe_angles.keys() for k in joint_angle_names[body_section]):
            params = {k: keyframe_angles[k] for k in joint_angle_names[body_section] if k in keyframe_angles.keys()}
        return params

    keyframe_proto = AnimationKeyframe()

    # Set the Keyframe time
    if "start_time" in keyframe_angles.keys():
        keyframe_proto.time = keyframe_angles["start_time"]
    if "time" in keyframe_angles.keys():
        keyframe_proto.time = keyframe_angles["time"]
    elif start_time is not None:
        keyframe_proto.time = start_time
    else:
        print("No start time specified, skipping joint angle keyframe")
        return None

    if params := extract_paramaters("gripper"):
        keyframe_proto.gripper.CopyFrom(build_gripper_params(**params))
    if params := extract_paramaters("arm"):
        keyframe_proto.arm.CopyFrom(build_arm_params(**params))
    if params := extract_paramaters("body"):
        keyframe_proto.body.CopyFrom(build_body_params(**params))

    front_left_params = extract_paramaters("front_left_leg")
    front_right_params = extract_paramaters("front_right_leg")
    rear_left_params = extract_paramaters("rear_left_leg")
    rear_right_params = extract_paramaters("rear_right_leg")

    if front_left_params and front_right_params and rear_left_params and rear_right_params:
        keyframe_proto.legs.CopyFrom(
            build_leg_params(**front_left_params, **front_right_params, **rear_left_params, **rear_right_params)
        )
    return keyframe_proto
