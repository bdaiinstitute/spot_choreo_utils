# Copyright (c) 2024-2025  Boston Dynamics AI Institute LLC. All rights reserved.

from pathlib import Path
from typing import Dict, Optional

from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    AnimationKeyframe,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context
from spot_web_animator.systems.spot import (
    MockSpot,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import (
    joint_angle_keyframe_to_proto,
)
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import save_animation


def new_animation(idle_keyframe: AnimationKeyframe, keyframe_time: int, keyframe_count: int) -> Animation:
    """Create a new animation to iteratively edit with pose-to-pose animation"""
    animation = Animation()
    animation_name = input("animation_name:")
    animation.name = animation_name
    animation.controls_gripper = True
    animation.controls_legs = True
    animation.controls_body = True
    animation.controls_arm = True
    idle_keyframe.time = 0
    animation.animation_keyframes.append(idle_keyframe)
    return animation


def save_pose(keyframe: AnimationKeyframe, keyframe_count: int, keyframe_time: float, animation: Animation) -> None:
    """Adds the pose to the active animation and saves it to disk"""
    keyframe_count += 1
    keyframe.time = keyframe_time
    animation.animation_keyframes.append(keyframe)

    path_to_animation = Path(get_active_choreo_path(), animation.name)

    # Everytime the save button is pressed, the animation is added to and then the document is overwritten
    save_animation(animation, path_to_animation)
    print("***********************Updated Animation********************")
    print(f"Updated Animation at: {path_to_animation}")


def print_current_pose_as_keyframe(animation_keyframe_map: dict[str, float]) -> None:
    """Print all joint angles as dictionary"""
    print(animation_keyframe_map)


def print_as_animation_keyframe(
    spot_plant: MultibodyPlant, spot: MockSpot, plant_context: Context, X_world_body: RigidTransform
) -> Dict:
    animation_keyframe_map = {}
    arm_joint_animation_names = ["shoulder_0", "shoulder_1", "elbow_0", "elbow_1", "wrist_0", "wrist_1"]
    arm_joint_names = ["arm0_sh0", "arm0_sh1", "arm0_el0", "arm0_el1", "arm0_wr0", "arm0_wr1"]
    for idx, joint in enumerate(arm_joint_names):
        joint_angle = spot_plant.GetJointByName(joint, spot.model.arm_instance).get_angle(plant_context)
        animation_keyframe_map[arm_joint_animation_names[idx]] = joint_angle

    body_joints_keyframe_name = [
        "front_left_hip_x",
        "front_left_hip_y",
        "front_left_knee",
        "front_right_hip_x",
        "front_right_hip_y",
        "front_right_knee",
        "rear_left_hip_x",
        "rear_left_hip_y",
        "rear_left_knee",
        "rear_right_hip_x",
        "rear_right_hip_y",
        "rear_right_knee",
    ]
    body_joints_drake_name = [
        "fl_hx",
        "fl_hy",
        "fl_kn",
        "fr_hx",
        "fr_hy",
        "fr_kn",
        "hl_hx",
        "hl_hy",
        "hl_kn",
        "hr_hx",
        "hr_hy",
        "hr_kn",
    ]

    for name_idx, joint_name in enumerate(body_joints_drake_name):
        joint_angle = spot_plant.GetJointByName(joint_name, spot.model.base_instance).get_angle(plant_context)
        animation_keyframe_map[body_joints_keyframe_name[name_idx]] = joint_angle

    # Add body poses and roation
    body_t = X_world_body.translation()
    body_q = X_world_body.rotation().ToQuaternion()
    animation_keyframe_map["body_pos_x"] = body_t[0]
    animation_keyframe_map["body_pos_y"] = body_t[1]
    animation_keyframe_map["body_pos_z"] = body_t[2]

    animation_keyframe_map["body_quat_w"] = body_q.w()
    animation_keyframe_map["body_quat_x"] = body_q.x()
    animation_keyframe_map["body_quat_y"] = body_q.y()
    animation_keyframe_map["body_quat_z"] = body_q.z()

    return animation_keyframe_map


def joint_angles_to_keyframe(animation_keyframe_map: Dict) -> Optional[AnimationKeyframe]:
    joint_angles = animation_keyframe_map
    joint_angles["start_time"] = 0
    return joint_angle_keyframe_to_proto(joint_angles)
