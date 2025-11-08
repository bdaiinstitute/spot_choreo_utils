# Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# Copyright (c) 2024-2025  Boston Dynamics AI Institute LLC. All rights reserved.

from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    AnimationKeyframe,
)
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Context
from spot_web_animator.systems.spot import (
    MockSpot,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_builder import AnimationBuilder
from spot_choreo_utils.choreo_creation.choreo_builders.animation_operators import (
    adjust_keyframes_to_timestamps,
)
from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import (
    joint_angle_keyframe_to_proto,
)
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import SemanticAnimationBuilder
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import load_animation, save_animation


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
    print(f"Created new animation '{animation.name}' with 1 initial keyframe")
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
    print(f"Added keyframe at time {keyframe_time}s (Total keyframes: {len(animation.animation_keyframes)})")


def update_keyframe(animation: Animation, index: int, keyframe: AnimationKeyframe) -> None:
    """Updates an existing keyframe in the animation"""
    if 0 <= index < len(animation.animation_keyframes):
        # Preserve the original time
        original_time = animation.animation_keyframes[index].time
        keyframe.time = original_time
        # Update the keyframe
        animation.animation_keyframes[index].CopyFrom(keyframe)
        print(f"Updated keyframe at index {index}, time {original_time}s")
    else:
        print(f"Invalid keyframe index: {index}")


def update_keyframe_time(animation: Animation, index: int, new_time: float) -> bool:
    """Updates the time of a keyframe, maintaining monotonically increasing timestamps"""
    if index < 0 or index >= len(animation.animation_keyframes):
        print(f"Invalid keyframe index: {index}")
        return False

    # Get the current time and surrounding keyframe times
    prev_time = animation.animation_keyframes[index - 1].time if index > 0 else 0
    next_time = (
        animation.animation_keyframes[index + 1].time
        if index < len(animation.animation_keyframes) - 1
        else float("inf")
    )

    # Check if new time would maintain monotonic increasing timestamps
    if new_time <= prev_time:
        print(f"Cannot set time to {new_time}s - must be greater than previous keyframe time ({prev_time}s)")
        return False

    if new_time >= next_time and index < len(animation.animation_keyframes) - 1:
        print(f"Cannot set time to {new_time}s - must be less than next keyframe time ({next_time}s)")
        return False

    # Set the new time
    original_time = animation.animation_keyframes[index].time
    animation.animation_keyframes[index].time = new_time
    print(f"Updated keyframe {index+1} time from {original_time}s to {new_time}s")
    return True


def create_semantic_animation_builder(animation: Animation) -> SemanticAnimationBuilder:
    """Creates a semantic animation builder from an animation"""
    semantic_builder = SemanticAnimationBuilder()
    semantic_builder.start_from_animation(animation)
    return semantic_builder


def name_animation_segment(
    semantic_builder: SemanticAnimationBuilder, segment_name: str, start_index: int, end_index: int
) -> bool:
    """Names a segment of the animation"""
    try:
        semantic_builder.name_segment_by_index(segment_name, start_index, end_index)
        return True
    except Exception as e:
        print(f"Error naming segment: {e}")
        return False


def save_semantic_animation(
    semantic_builder: SemanticAnimationBuilder, output_folder: Optional[Path] = None
) -> Optional[Path]:
    """Saves a semantic animation to disk"""
    if output_folder is None:
        animation = semantic_builder.animation_builder.raw_animation
        output_folder = Path(get_active_choreo_path(), animation.name)

    return semantic_builder.save_semantic_animation(output_folder)


def adjust_animation_timing(animation: Animation, keyframe_indices: List[int], new_timestamps: List[float]) -> bool:
    """
    Adjusts the timing of keyframes, using animation_operators.adjust_keyframes_to_timestamps.
    This maintains proper spacing between all keyframes.

    Args:
        animation: The animation to modify
        keyframe_indices: List of keyframe indices to adjust
        new_timestamps: New timestamps for each keyframe in keyframe_indices

    Returns:
        True if successful, False otherwise
    """
    if len(keyframe_indices) != len(new_timestamps):
        print("Error: Number of keyframe indices must match number of timestamps")
        return False

    # Check that indices and timestamps are monotonically increasing
    if not all(keyframe_indices[i] < keyframe_indices[i + 1] for i in range(len(keyframe_indices) - 1)):
        print("Error: Keyframe indices must be monotonically increasing")
        return False

    if not all(new_timestamps[i] < new_timestamps[i + 1] for i in range(len(new_timestamps) - 1)):
        print("Error: New timestamps must be monotonically increasing")
        return False

    # Create a builder to use the animation operators
    builder = AnimationBuilder()
    builder.start_from_animation(animation)

    try:
        # Apply the time adjustment
        adjust_keyframes_to_timestamps(builder, keyframe_indices, new_timestamps)

        # Copy the adjusted keyframes back to the original animation
        del animation.animation_keyframes[:]
        animation.animation_keyframes.extend(builder.raw_keyframes)

        print(f"Successfully adjusted keyframe timing for {len(keyframe_indices)} keyframes")
        return True
    except Exception as e:
        print(f"Error adjusting keyframe timing: {e}")
        return False


def get_keyframe_times(animation: Animation) -> List[float]:
    """Returns a list of all keyframe times in the animation"""
    return [keyframe.time for keyframe in animation.animation_keyframes]


def load_animation_from_file(path: Path) -> Optional[Animation]:
    """Loads an animation from a file"""
    animation = load_animation(path)
    if animation:
        print(f"Loaded animation '{animation.name}' with {len(animation.animation_keyframes)} keyframes")
    else:
        print(f"Failed to load animation from {path}")

    return animation


def print_current_pose_as_keyframe(animation_keyframe_map: dict[str, float]) -> None:
    """Print all joint angles as dictionary"""
    print(animation_keyframe_map)


def print_as_animation_keyframe(
    spot_plant: MultibodyPlant, spot: MockSpot, plant_context: Context, X_world_body: RigidTransform
) -> Dict:
    animation_keyframe_map = {}

    animation_keyframe_map["gripper"] = spot_plant.GetJointByName("arm_f1x", spot.model.gripper_instance).get_angle(
        plant_context
    )

    arm_joint_animation_names = ["shoulder_0", "shoulder_1", "elbow_0", "elbow_1", "wrist_0", "wrist_1"]
    arm_joint_names = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1"]
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


def load_joint_angles_from_keyframe(keyframe: AnimationKeyframe, spot: MockSpot, plant_context: Context) -> None:
    """Applies the joint angles from a keyframe to the robot model"""
    spot_plant = spot.model.get_mutable_multibody_plant()

    # Get current positions
    spot_plant.GetPositions(plant_context)

    # Update body pose if available
    if keyframe.HasField("body"):
        body = keyframe.body
        if body.HasField("body_pos") and body.HasField("quaternion"):
            # Create transform from pose components
            body_pos = body.body_pos
            quat = body.quaternion

            x = body_pos.x.value if body_pos.HasField("x") else 0.0
            y = body_pos.y.value if body_pos.HasField("y") else 0.0
            z = body_pos.z.value if body_pos.HasField("z") else 0.0

            # Set the free body pose using the proper constructor format
            # Normalize the quaternion
            qw, qx, qy, qz = quat.w, quat.x, quat.y, quat.z
            magnitude = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
            if magnitude > 0:
                qw /= magnitude
                qx /= magnitude
                qy /= magnitude
                qz /= magnitude
            else:
                # If zero magnitude, use identity quaternion
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

            X_WB = RigidTransform(Quaternion(qw, qx, qy, qz), np.array([x, y, z]))
            spot.model.SetFreeBodyPose(plant_context, X_WB)

    # Update arm positions if available
    if spot.model.has_arm and keyframe.HasField("arm") and keyframe.arm.HasField("joint_angles"):
        arm_joint_angles = keyframe.arm.joint_angles
        arm_positions = spot.model.get_arm_state(plant_context)[: spot.model.num_arm_positions()]

        # Map from KeyFrame names to indices
        arm_joint_map = {"shoulder_0": 0, "shoulder_1": 1, "elbow_0": 2, "elbow_1": 3, "wrist_0": 4, "wrist_1": 5}

        # Update arm joint positions
        if arm_joint_angles.HasField("shoulder_0"):
            arm_positions[arm_joint_map["shoulder_0"]] = arm_joint_angles.shoulder_0.value
        if arm_joint_angles.HasField("shoulder_1"):
            arm_positions[arm_joint_map["shoulder_1"]] = arm_joint_angles.shoulder_1.value
        if arm_joint_angles.HasField("elbow_0"):
            arm_positions[arm_joint_map["elbow_0"]] = arm_joint_angles.elbow_0.value
        if arm_joint_angles.HasField("elbow_1"):
            arm_positions[arm_joint_map["elbow_1"]] = arm_joint_angles.elbow_1.value
        if arm_joint_angles.HasField("wrist_0"):
            arm_positions[arm_joint_map["wrist_0"]] = arm_joint_angles.wrist_0.value
        if arm_joint_angles.HasField("wrist_1"):
            arm_positions[arm_joint_map["wrist_1"]] = arm_joint_angles.wrist_1.value

        # Apply the updated arm positions
        spot_plant.SetPositions(plant_context, spot.model.arm_instance, arm_positions)

    # Update gripper position if available
    if spot.model.has_arm and keyframe.HasField("gripper") and keyframe.gripper.HasField("gripper_angle"):
        gripper_positions = spot.model.get_gripper_state(plant_context)[: spot.model.num_gripper_positions()]
        gripper_positions[0] = keyframe.gripper.gripper_angle.value
        spot_plant.SetPositions(plant_context, spot.model.gripper_instance, gripper_positions)

    # Update leg positions if available
    if keyframe.HasField("legs"):
        legs = keyframe.legs

        # Front left leg
        if legs.HasField("fl") and legs.fl.HasField("joint_angles"):
            fl = legs.fl.joint_angles
            spot_plant.GetJointByName("front_left_hip_x", spot.model.base_instance).set_angle(plant_context, fl.hip_x)
            spot_plant.GetJointByName("front_left_hip_y", spot.model.base_instance).set_angle(plant_context, fl.hip_y)
            spot_plant.GetJointByName("front_left_knee", spot.model.base_instance).set_angle(plant_context, fl.knee)

        # Front right leg
        if legs.HasField("fr") and legs.fr.HasField("joint_angles"):
            fr = legs.fr.joint_angles
            spot_plant.GetJointByName("front_right_hip_x", spot.model.base_instance).set_angle(plant_context, fr.hip_x)
            spot_plant.GetJointByName("front_right_hip_y", spot.model.base_instance).set_angle(plant_context, fr.hip_y)
            spot_plant.GetJointByName("front_right_knee", spot.model.base_instance).set_angle(plant_context, fr.knee)

        # Rear left leg
        if legs.HasField("hl") and legs.hl.HasField("joint_angles"):
            hl = legs.hl.joint_angles
            spot_plant.GetJointByName("rear_left_hip_x", spot.model.base_instance).set_angle(plant_context, hl.hip_x)
            spot_plant.GetJointByName("rear_left_hip_y", spot.model.base_instance).set_angle(plant_context, hl.hip_y)
            spot_plant.GetJointByName("rear_left_knee", spot.model.base_instance).set_angle(plant_context, hl.knee)

        # Rear right leg
        if legs.HasField("hr") and legs.hr.HasField("joint_angles"):
            hr = legs.hr.joint_angles
            spot_plant.GetJointByName("rear_right_hip_x", spot.model.base_instance).set_angle(plant_context, hr.hip_x)
            spot_plant.GetJointByName("rear_right_hip_y", spot.model.base_instance).set_angle(plant_context, hr.hip_y)
            spot_plant.GetJointByName("rear_right_knee", spot.model.base_instance).set_angle(plant_context, hr.knee)
