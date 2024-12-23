# Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

from itertools import chain
from typing import Dict, List, Tuple

import numpy as np

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import protobuf_joint_angle_map


def get_flattened_joint_angle_names() -> Dict[str, List[str]]:
    """Returns a map of which joint angles are part of each protobuf paramaterization"""
    joint_angle = protobuf_joint_angle_map()
    return list(chain.from_iterable(joint_angle.values()))


def get_joint_angle_names() -> Dict[str, List[str]]:
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


def joint_angle_to_protobuf_attrs_map() -> dict[str, List[str]]:
    """Explicit flattened joint names to protobuf attributes"""
    attrs_map = {}
    joint_angle_names = protobuf_joint_angle_map()

    # Gripper
    attrs_map["gripper"] = ["gripper", "gripper_angle", "value"]

    # Arms
    arm_angle_names = joint_angle_names["arm"]
    for arm_joint_name in arm_angle_names:
        attrs_map[arm_joint_name] = ["arm", "joint_angles", arm_joint_name, "value"]

    # Body
    body_angle_names = joint_angle_names["body"]
    body_position_name = body_angle_names[:3]
    body_quaternion_names = body_angle_names[3:]
    for body_pos_name in body_position_name:
        attrs_map[body_pos_name] = ["body", "body_pos", body_pos_name[-1], "value"]

    for body_quat_name in body_quaternion_names:
        attrs_map[body_quat_name] = ["body", "quaternion", body_quat_name[-1]]

    # Legs
    leg_keys = {"front_left_leg": "fl", "front_right_leg": "fr", "rear_left_leg": "hl", "rear_right_leg": "hr"}
    for current_animation_leg_key, proto_key in leg_keys.items():
        specific_leg_names = joint_angle_names[current_animation_leg_key]
        for individual_leg_joint_names in specific_leg_names:
            proto_joint_name = individual_leg_joint_names.replace(current_animation_leg_key[:-3], "")
            attrs_map[individual_leg_joint_names] = ["legs", proto_key, "joint_angles", proto_joint_name]

    return attrs_map


def get_joint_limits() -> Dict[str, Tuple[float, float]]:
    """Returns a map of the min/max angle for each joint"""

    def to_rad(th: float) -> float:
        return th * np.pi / 180

    all_joints = {}

    # Set gripper limits
    gripper_dict = {"gripper": (-1, 0)}
    all_joints.update(gripper_dict)

    # Set leg limits
    leg_limits = [(to_rad(-45), to_rad(45)), (to_rad(-51.5), to_rad(131.5)), (to_rad(-14.2), to_rad(160))]

    joint_angle_names = protobuf_joint_angle_map()
    fll_labels = joint_angle_names["front_left_leg"]
    frl_labels = joint_angle_names["front_right_leg"]
    rll_labels = joint_angle_names["rear_left_leg"]
    rrl_labels = joint_angle_names["rear_right_leg"]

    all_joints.update(dict(zip(fll_labels, leg_limits, strict=True)))
    all_joints.update(dict(zip(frl_labels, leg_limits, strict=True)))
    all_joints.update(dict(zip(rll_labels, leg_limits, strict=True)))
    all_joints.update(dict(zip(rrl_labels, leg_limits, strict=True)))

    # Set arm limits
    arm_joint_names = joint_angle_names["arm"]
    arm_limits = [
        (to_rad(-150), to_rad(180)),
        (to_rad(-180), to_rad(30)),
        (0, to_rad(180)),
        (to_rad(-160), to_rad(160)),
        (to_rad(-105), to_rad(105)),
        (to_rad(-165), to_rad(165)),
    ]

    all_joints.update(dict(zip(arm_joint_names, arm_limits, strict=True)))
    return all_joints
