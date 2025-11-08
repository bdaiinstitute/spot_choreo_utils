# Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# Copyright (c) 2024-2025  Boston Dynamics AI Institute LLC. All rights reserved.

import asyncio
import time as time_module
from copy import copy
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

import numpy as np
import numpy.typing as npt
from pydrake.geometry import Meshcat
from pydrake.geometry.optimization import HPolyhedron, VPolytope
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.multibody.inverse_kinematics import (
    InverseKinematics,
)
from pydrake.solvers import Solve
from pydrake.systems.framework import (
    Context,
)
from api.spot_animator_connector import app

from spot_web_animator.animation_backend.web_animator_drake_ui import (
    ButtonNames,
    display_save_button,
    setup_default_drake_robot_meshcat,
    setup_ui_buttons,
    update_keyframe_counter,
)
from spot_web_animator.animation_backend.web_animator_serializer import (
    adjust_animation_timing,
    create_semantic_animation_builder,
    get_keyframe_times,
    joint_angles_to_keyframe,
    load_animation_from_file,
    load_joint_angles_from_keyframe,
    name_animation_segment,
    new_animation,
    print_as_animation_keyframe,
    print_current_pose_as_keyframe,
    save_pose,
    save_semantic_animation,
    update_keyframe,
    update_keyframe_time,
)
from spot_web_animator.systems.spot import (
    FootCenterFrame,
    SpotModel,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import (
    joint_angle_keyframe_to_proto,
)
from spot_choreo_utils.choreo_creation.semantic_animations.semantic_animation_builder import (
    SemanticAnimationBuilder
)
from spot_choreo_utils.choreo_playback.synced_audio_player import SyncedAudioPlayer
from spot_choreo_utils.choreo_playback.synced_performance_coordinator import SyncedPerformanceCoordinator
from spot_choreo_utils.choreo_playback.synced_performance_modality import SyncedPeroformanceConfig
from spot_choreo_utils.choreo_playback.synced_spot_dancer import SyncedSpotDancer
from spot_choreo_utils.paths import get_active_choreo_path
from spot_choreo_utils.serialization.serialization_utils import save_animation

# Derived manually from the URDF
leg_limits_base = [
    [-0.78539816339744827899, 0.78539816339744827899],
    [-0.89884456477707963539, 2.2951079663725435509],
    [-2.7929, -0.254402],
]

idle_joint_angles = {
    "shoulder_0": 0.0,
    "shoulder_1": -3.12,
    "elbow_0": 3.13,
    "elbow_1": 1.57,
    "wrist_0": -0.00001,
    "wrist_1": -1.57,
    "gripper": -0.054,
    "front_left_hip_x": 0.12,
    "front_left_hip_y": 0.72,
    "front_left_knee": -1.45,
    "front_right_hip_x": -0.12,
    "front_right_hip_y": 0.72,
    "front_right_knee": -1.45,
    "rear_left_hip_x": 0.12,
    "rear_left_hip_y": 0.72,
    "rear_left_knee": -1.45,
    "rear_right_hip_x": -0.12,
    "rear_right_hip_y": 0.72,
    "rear_right_knee": -1.45,
    "body_pos_x": 0.0,
    "body_pos_y": 0.0,
    "body_pos_z": 0.525,
    "body_quat_w": 1.0,
    "body_quat_x": 0.0,
    "body_quat_y": 0.0,
    "body_quat_z": 0.0,
}

idle_joint_angles["start_time"] = 0

def spot_body_inverse_kinematics(
    feet_on_floor: bool,
    unlocked_legs: List[FootCenterFrame],
    X_world_body: RigidTransform,
    spot_model: SpotModel,
    plant_context: Context,
    support_polygon_margin: float = 0.1,
) -> Optional[npt.NDArray]:
    """Computes the inverse kinematics to achieve a desired body pose.

    The center of mass will be constraint to be within the support polygon.
    All feet will be constraint at the given positions in the plant context.

    Args:
        unlocked_legs: array of which legs should be locked or unlocked
        X_world_body: The desired pose of the body wrt to the world frame.
        spot_model: The spot robot model with or without an arm used for the ik.
        plant_context: Extracts the current feet positions from the plant context
            for position constraints
        support_polygon_margin: Can be in [0, 1], which controls the safety margin
            where the center of mass is allowed in. E.g. 0.1 will shrink the support
            polygon by 10%.

    Returns:
        The ik solution of the joint configurations for the whole plant.
    """

    MINIMUM_BODY_HEIGHT = 0.1  # in meters.
    MAXIMUM_BODY_HEIGHT = 0.6 
    spot_plant = spot_model.get_mutable_multibody_plant()
    body_frame = spot_plant.GetFrameByName("body", spot_model.base_instance)

    ik = InverseKinematics(spot_plant, plant_context)

    if feet_on_floor:
        for frame_name in FootCenterFrame:
            if frame_name in unlocked_legs:
                foot_frame = spot_plant.GetFrameByName(frame_name.value, spot_model.base_instance)
                foot_position = foot_frame.CalcPoseInWorld(plant_context).translation()

                ik.AddPositionConstraint(
                    foot_frame,
                    [0, 0, 0],
                    spot_plant.world_frame(),
                    [foot_position[0], foot_position[1], 0],
                    [foot_position[0], foot_position[1], 0],
                )

    # Add position and orientation constraint for the base body.
    ik.AddPositionConstraint(
        body_frame,
        [0, 0, 0],
        spot_plant.world_frame(),
        X_world_body.translation(),
        X_world_body.translation(),
    )
    ik.AddOrientationConstraint(
        body_frame,
        RotationMatrix(),
        spot_plant.world_frame(),
        X_world_body.rotation(),
        0.001,
    )

    prog = ik.get_mutable_prog()
    q = ik.q()

    # Incentivize the IK solver to find a solution near the
    # current joint configuration.
    q0 = spot_plant.GetPositions(plant_context)
    prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
    prog.SetInitialGuess(q, q0)

    support_polygon_points = []
    # Ensure that feet centers don't move.
    for frame_name in FootCenterFrame:
        if frame_name in unlocked_legs and not feet_on_floor:
            print(f"removed {frame_name}")
            continue

        foot_frame = spot_plant.GetFrameByName(frame_name.value, spot_model.base_instance)
        foot_position = foot_frame.CalcPoseInWorld(plant_context).translation()
        ik.AddPositionConstraint(
            foot_frame,
            np.array([0, 0, 0]),
            spot_plant.world_frame(),
            foot_position,
            foot_position,
        )

        support_polygon_points.append(foot_position)

    if len(support_polygon_points) >= 1:
        support_polygon_points = np.vstack(support_polygon_points)

        # Scale support polygon at its center.
        support_polygon_center = support_polygon_points.mean(axis=0)  # type: ignore
        support_polygon_points -= support_polygon_center
        support_polygon_points *= 1 - support_polygon_margin  # type: ignore
        support_polygon_points += support_polygon_center

        # Use the support polygon points to compute the center of mass
        # polyhedron constraint.
        lower_com_polyhedra_points = support_polygon_points.copy()
        upper_com_polyhedra_points = support_polygon_points.copy()
        lower_com_polyhedra_points[:, 2] = MINIMUM_BODY_HEIGHT  # type: ignore
        upper_com_polyhedra_points[:, 2] = MAXIMUM_BODY_HEIGHT  # type: ignore

        HPolyhedron(VPolytope(np.concatenate([lower_com_polyhedra_points, upper_com_polyhedra_points]).T))

    # # Add position constraints of the leg joints.
    q_legs = q[7 : spot_model.num_base_positions()]

    prog.AddBoundingBoxConstraint(
        spot_plant.GetPositionLowerLimits()[7 : spot_model.num_base_positions()],
        spot_plant.GetPositionUpperLimits()[7 : spot_model.num_base_positions()],
        q_legs,
    )
    if spot_model.has_arm:
        # Constrain the arm and gripper not to move.
        q0_arm = spot_model.get_arm_state(plant_context)[: spot_model.num_arm_positions()]
        q_arm = q[spot_model.num_base_positions() : spot_model.num_base_positions() + spot_model.num_arm_positions()]
        prog.AddLinearEqualityConstraint(np.diag(np.ones(spot_model.num_arm_positions())), q0_arm, q_arm)

        q0_gripper = spot_model.get_gripper_state(plant_context)[: spot_model.num_gripper_positions()]
        q_gripper = q[spot_model.num_base_positions() + spot_model.num_arm_positions() :]
        prog.AddLinearEqualityConstraint(
            np.diag(np.ones(spot_model.num_gripper_positions())),
            q0_gripper,
            q_gripper,
        )


    result = Solve(prog)

    if not result.is_success():
        return None

    spot_plant.SetPositions(plant_context, result.GetSolution(q))
    return result.GetSolution(q)


def update_robot_config_from_joint_angles(joint_angles, spot, plant_context):
    """Updates the robot configuration in the Meshcat visualization based on joint angles"""
    spot_plant = spot.model.get_mutable_multibody_plant()
    
    # For arm joints - update positions directly
    if spot.model.has_arm:
        # Get current arm positions array
        arm_positions = spot.model.get_arm_state(plant_context)[:spot.model.num_arm_positions()]
        
        # Map from frontend names to indices in arm_positions array
        arm_indices = {
            "shoulder_0": 0,
            "shoulder_1": 1,
            "elbow_0": 2,
            "elbow_1": 3,
            "wrist_0": 4,
            "wrist_1": 5
        }
        
        # Update the arm positions array with values from joint_angles
        arm_updated = False
        for frontend_name, index in arm_indices.items():
            if frontend_name in joint_angles and index < len(arm_positions):
                arm_positions[index] = joint_angles[frontend_name]
                arm_updated = True
                print(f"Setting arm joint {frontend_name} to {joint_angles[frontend_name]}")
        
        # Apply the updated arm positions array
        if arm_updated:
            print(f"Applying arm positions: {arm_positions}")
            spot_plant.SetPositions(plant_context, spot.model.arm_instance, arm_positions)
    
    # For gripper - update similarly
    if spot.model.has_arm and "gripper" in joint_angles:
        gripper_positions = spot.model.get_gripper_state(plant_context)[:spot.model.num_gripper_positions()]
        gripper_positions[0] = joint_angles["gripper"]
        spot_plant.SetPositions(plant_context, spot.model.gripper_instance, gripper_positions)
    
    # Legs - keep the existing approach
    leg_mappings = {
        "front_left_hip_x": ("front_left_hip_x", spot.model.base_instance),
        "front_left_hip_y": ("front_left_hip_y", spot.model.base_instance),
        "front_left_knee": ("front_left_knee", spot.model.base_instance),
        "front_right_hip_x": ("front_right_hip_x", spot.model.base_instance),
        "front_right_hip_y": ("front_right_hip_y", spot.model.base_instance),
        "front_right_knee": ("front_right_knee", spot.model.base_instance),
        "rear_left_hip_x": ("rear_left_hip_x", spot.model.base_instance),
        "rear_left_hip_y": ("rear_left_hip_y", spot.model.base_instance),
        "rear_left_knee": ("rear_left_knee", spot.model.base_instance),
        "rear_right_hip_x": ("rear_right_hip_x", spot.model.base_instance),
        "rear_right_hip_y": ("rear_right_hip_y", spot.model.base_instance),
        "rear_right_knee": ("rear_right_knee", spot.model.base_instance),
    }
    
    for frontend_name, (backend_name, instance) in leg_mappings.items():
        if frontend_name in joint_angles:
            try:
                spot_plant.GetJointByName(backend_name, instance).set_angle(
                    plant_context, joint_angles[frontend_name]
                )
            except Exception as e:
                print(f"Error setting angle for {backend_name}: {e}")
    
    if all(k in joint_angles for k in ["body_pos_x", "body_pos_y", "body_pos_z", 
                                       "body_quat_w", "body_quat_x", "body_quat_y", "body_quat_z"]):
        # Create quaternion with proper normalization
        try:
            # Extract quaternion components
            w = joint_angles["body_quat_w"]
            x = joint_angles["body_quat_x"]
            y = joint_angles["body_quat_y"]
            z = joint_angles["body_quat_z"]
            
            # Calculate magnitude for normalization
            magnitude = np.sqrt(w*w + x*x + y*y + z*z)
            
            # Normalize quaternion if magnitude is not zero
            if magnitude > 1e-10:  # Avoid division by very small numbers
                w /= magnitude
                x /= magnitude
                y /= magnitude
                z /= magnitude
            else:
                # Default to identity quaternion if magnitude is too small
                w, x, y, z = 1.0, 0.0, 0.0, 0.0
                
            quat = Quaternion(w, x, y, z)
            
            X_WB = RigidTransform(
                quat,
                np.array([
                    joint_angles["body_pos_x"],
                    joint_angles["body_pos_y"],
                    joint_angles["body_pos_z"]
                ])
            )
            spot.model.SetFreeBodyPose(plant_context, X_WB)
        except Exception as e:
            print(f"Error setting body pose: {e}")
            return False
    
    return True


def print_keyframe_info(keyframe, current_index, total_count):
    """Prints detailed information about the current keyframe"""
    print(f"\n=== KEYFRAME {current_index + 1}/{total_count} ===")
    print(f"Time: {keyframe.time:.2f}s")
    
    # Display joint angles if available
    print("\nJoint Angles:")
    
    if keyframe.HasField("gripper") and keyframe.gripper.HasField("gripper_angle"):
        print(f"  gripper: {keyframe.gripper.gripper_angle.value:.4f}")
    
    if keyframe.HasField("arm") and keyframe.arm.HasField("joint_angles"):
        joint_angles = keyframe.arm.joint_angles
        if joint_angles.HasField("shoulder_0"):
            print(f"  shoulder_0: {joint_angles.shoulder_0.value:.4f}")
        if joint_angles.HasField("shoulder_1"):
            print(f"  shoulder_1: {joint_angles.shoulder_1.value:.4f}")
        if joint_angles.HasField("elbow_0"):
            print(f"  elbow_0: {joint_angles.elbow_0.value:.4f}")
        if joint_angles.HasField("elbow_1"):
            print(f"  elbow_1: {joint_angles.elbow_1.value:.4f}")
        if joint_angles.HasField("wrist_0"):
            print(f"  wrist_0: {joint_angles.wrist_0.value:.4f}")
        if joint_angles.HasField("wrist_1"):
            print(f"  wrist_1: {joint_angles.wrist_1.value:.4f}")
    
    # Display body pose if available
    if keyframe.HasField("body"):
        print("\nBody Pose:")
        body = keyframe.body
        if body.HasField("body_pos"):
            pos = body.body_pos
            if pos.HasField("x"):
                print(f"  position_x: {pos.x.value:.4f}")
            if pos.HasField("y"):
                print(f"  position_y: {pos.y.value:.4f}")
            if pos.HasField("z"):
                print(f"  position_z: {pos.z.value:.4f}")
        
        if body.HasField("quaternion"):
            quat = body.quaternion
            print(f"  quaternion_w: {quat.w:.4f}")
            print(f"  quaternion_x: {quat.x:.4f}")
            print(f"  quaternion_y: {quat.y:.4f}")
            print(f"  quaternion_z: {quat.z:.4f}")
    
    # Display leg angles if available
    if keyframe.HasField("legs"):
        print("\nLeg Angles:")
        legs = keyframe.legs
        
        if legs.HasField("fl") and legs.fl.HasField("joint_angles"):
            fl = legs.fl.joint_angles
            print(f"  front_left_hip_x: {fl.hip_x:.4f}")
            print(f"  front_left_hip_y: {fl.hip_y:.4f}")
            print(f"  front_left_knee: {fl.knee:.4f}")
        
        if legs.HasField("fr") and legs.fr.HasField("joint_angles"):
            fr = legs.fr.joint_angles
            print(f"  front_right_hip_x: {fr.hip_x:.4f}")
            print(f"  front_right_hip_y: {fr.hip_y:.4f}")
            print(f"  front_right_knee: {fr.knee:.4f}")
        
        if legs.HasField("hl") and legs.hl.HasField("joint_angles"):
            hl = legs.hl.joint_angles
            print(f"  rear_left_hip_x: {hl.hip_x:.4f}")
            print(f"  rear_left_hip_y: {hl.hip_y:.4f}")
            print(f"  rear_left_knee: {hl.knee:.4f}")
        
        if legs.HasField("hr") and legs.hr.HasField("joint_angles"):
            hr = legs.hr.joint_angles
            print(f"  rear_right_hip_x: {hr.hip_x:.4f}")
            print(f"  rear_right_hip_y: {hr.hip_y:.4f}")
            print(f"  rear_right_knee: {hr.knee:.4f}")


# Button handler functions
def handle_save_keyframe(
    animation, 
    animation_keyframe_map, 
    keyframe_count, 
    keyframe_time, 
    meshcat, 
    button_names, 
    current_keyframe_index
) -> Tuple[int, int, int]:
    """Handle saving current pose as a keyframe"""
    keyframe = joint_angles_to_keyframe(animation_keyframe_map)
    keyframe_time += 1
    save_pose(keyframe, keyframe_count, keyframe_time, animation)
    
    # Update the keyframe counter after adding a new keyframe
    current_keyframe_index = len(animation.animation_keyframes) - 1
    update_keyframe_counter(meshcat, button_names, current_keyframe_index, len(animation.animation_keyframes))
    
    # Update the segment start/end index sliders to match the new animation length
    max_idx = len(animation.animation_keyframes) - 1
    meshcat.DeleteSlider(button_names.segment_start_idx_input)
    meshcat.DeleteSlider(button_names.segment_end_idx_input)
    meshcat.AddSlider(button_names.segment_start_idx_input, 0, max_idx, 1, 0)
    meshcat.AddSlider(button_names.segment_end_idx_input, 0, max_idx, 1, max_idx)
    
    return keyframe_time, keyframe_count, current_keyframe_index


def handle_prev_keyframe(
    animation, 
    meshcat, 
    button_names, 
    current_keyframe_index, 
    spot, 
    plant_context, 
    context, 
    body_pose_sliders
) -> Tuple[int, Dict[str, Any], RigidTransform]:
    """Handle previous keyframe button click"""
    if len(animation.animation_keyframes) > 0:
        current_keyframe_index = max(0, current_keyframe_index - 1)
        keyframe = animation.animation_keyframes[current_keyframe_index]
        update_keyframe_counter(meshcat, button_names, current_keyframe_index, len(animation.animation_keyframes))
        print(f"Selected keyframe {current_keyframe_index + 1}/{len(animation.animation_keyframes)} (Time: {keyframe.time:.2f}s)")
        
        # Update the time slider to match the keyframe time
        meshcat.SetSliderValue(button_names.keyframe_time_slider_name, keyframe.time)
        
        # Load the keyframe joint angles into the robot model
        load_joint_angles_from_keyframe(keyframe, spot, plant_context)
        spot.ForcedPublish(context)
        
        # Update X_world_body for IK calculations
        spot_plant = spot.model.get_mutable_multibody_plant()
        X_world_body = spot_plant.EvalBodyPoseInWorld(
            plant_context, spot_plant.GetBodyByName("body", spot.model.base_instance)
        )
        
        # Update animation keyframe map with the loaded keyframe
        animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
        
        # Update body pose sliders to match the loaded keyframe
        body_pose_sliders_context = body_pose_sliders.GetMyContextFromRoot(context)
        body_pose_sliders.SetPose(X_world_body)
        
        return current_keyframe_index, animation_keyframe_map, X_world_body
    
    return current_keyframe_index, None, None


def handle_next_keyframe(
    animation, 
    meshcat, 
    button_names, 
    current_keyframe_index, 
    spot, 
    plant_context, 
    context, 
    body_pose_sliders
) -> Tuple[int, Dict[str, Any], RigidTransform]:
    """Handle next keyframe button click"""
    if len(animation.animation_keyframes) > 0:
        current_keyframe_index = min(len(animation.animation_keyframes) - 1, current_keyframe_index + 1)
        keyframe = animation.animation_keyframes[current_keyframe_index]
        update_keyframe_counter(meshcat, button_names, current_keyframe_index, len(animation.animation_keyframes))
        print(f"Selected keyframe {current_keyframe_index + 1}/{len(animation.animation_keyframes)} (Time: {keyframe.time:.2f}s)")
        
        # Update the time slider to match the keyframe time
        meshcat.SetSliderValue(button_names.keyframe_time_slider_name, keyframe.time)
        
        # Load the keyframe joint angles into the robot model
        load_joint_angles_from_keyframe(keyframe, spot, plant_context)
        spot.ForcedPublish(context)
        
        # Update X_world_body for IK calculations
        spot_plant = spot.model.get_mutable_multibody_plant()
        X_world_body = spot_plant.EvalBodyPoseInWorld(
            plant_context, spot_plant.GetBodyByName("body", spot.model.base_instance)
        )
        
        # Update animation keyframe map with the loaded keyframe
        animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
        
        # Update body pose sliders to match the loaded keyframe
        body_pose_sliders_context = body_pose_sliders.GetMyContextFromRoot(context)
        body_pose_sliders.SetPose(X_world_body)
        
        return current_keyframe_index, animation_keyframe_map, X_world_body
    
    return current_keyframe_index, None, None


def handle_keyframe_info(animation, current_keyframe_index):
    """Handle keyframe info button click"""
    if len(animation.animation_keyframes) > 0:
        keyframe = animation.animation_keyframes[current_keyframe_index]
        print_keyframe_info(keyframe, current_keyframe_index, len(animation.animation_keyframes))
    else:
        print("No keyframes in animation")


def handle_update_keyframe(animation, current_keyframe_index, animation_keyframe_map, meshcat, button_names, semantic_builder):
    """Handle update keyframe button click"""
    if len(animation.animation_keyframes) > 0:
        # Get current joint angles
        current_joint_angles = animation_keyframe_map
        
        # Create a new keyframe with current joint angles
        updated_keyframe = joint_angles_to_keyframe(current_joint_angles)
        
        new_time = meshcat.GetSliderValue(button_names.keyframe_time_slider_name)
        
        # Update the keyframe in the animation with both new pose and time
        update_keyframe(animation, current_keyframe_index, updated_keyframe)
        
        # Update the time separately
        if update_keyframe_time(animation, current_keyframe_index, new_time):
            # Save the animation to disk
            path_to_animation = Path(get_active_choreo_path(), animation.name)
            save_animation(animation, path_to_animation)
            
            print(f"✓ Updated keyframe {current_keyframe_index + 1}/{len(animation.animation_keyframes)} (Time: {new_time:.2f}s)")
            
            # If we have a semantic builder, update it with the modified animation
            if semantic_builder is not None:
                semantic_builder.start_from_animation(animation)
                save_semantic_animation(semantic_builder)
        else:
            # Still save the keyframe values even if time update failed
            path_to_animation = Path(get_active_choreo_path(), animation.name)
            save_animation(animation, path_to_animation)
            print(f"✓ Updated keyframe {current_keyframe_index + 1}/{len(animation.animation_keyframes)} (Pose only)")
    else:
        print("No keyframes to update")


def handle_load_keyframe(
    animation, 
    current_keyframe_index, 
    spot, 
    plant_context, 
    context, 
    meshcat, 
    button_names, 
    body_pose_sliders
) -> Dict[str, Any]:
    """Handle load keyframe button click"""
    if len(animation.animation_keyframes) > 0:
        keyframe = animation.animation_keyframes[current_keyframe_index]
        
        # Load the keyframe joint angles into the robot model
        load_joint_angles_from_keyframe(keyframe, spot, plant_context)
        spot.ForcedPublish(context)
        
        # Update X_world_body for IK calculations
        spot_plant = spot.model.get_mutable_multibody_plant()
        X_world_body = spot_plant.EvalBodyPoseInWorld(
            plant_context, spot_plant.GetBodyByName("body", spot.model.base_instance)
        )
        
        # Update animation keyframe map with the loaded keyframe
        animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
        
        # Update body pose sliders to match the loaded keyframe
        body_pose_sliders_context = body_pose_sliders.GetMyContextFromRoot(context)
        body_pose_sliders.SetPose(X_world_body)
        
        # Update the time slider to match the keyframe time
        meshcat.SetSliderValue(button_names.keyframe_time_slider_name, keyframe.time)
        
        print(f"✓ Loaded keyframe {current_keyframe_index + 1}/{len(animation.animation_keyframes)} (Time: {keyframe.time:.2f}s)")
        
        return animation_keyframe_map
    else:
        print("No keyframes to load")
        return None


def handle_name_segment(animation, meshcat, button_names, semantic_builder):
    """Handle naming a segment of the animation"""
    if len(animation.animation_keyframes) > 0:
        # Get the segment name from console input
        segment_name = input("Enter segment name: ")
        
        # Get the start and end indices from the sliders
        start_index = int(meshcat.GetSliderValue(button_names.segment_start_idx_input))
        end_index = int(meshcat.GetSliderValue(button_names.segment_end_idx_input))
        
        # Validate the indices
        if start_index < 0 or start_index >= len(animation.animation_keyframes):
            print(f"Invalid start index: {start_index}")
        elif end_index < 0 or end_index >= len(animation.animation_keyframes):
            print(f"Invalid end index: {end_index}")
        elif start_index > end_index:
            print(f"Start index ({start_index}) must be less than or equal to end index ({end_index})")
        elif not segment_name:
            print("Segment name cannot be empty")
        else:
            # Create semantic builder if it doesn't exist
            if semantic_builder is None:
                semantic_builder = create_semantic_animation_builder(animation)
            
            # Name the segment
            if name_animation_segment(semantic_builder, segment_name, start_index, end_index):
                # Save the semantic animation
                semantic_file_path = save_semantic_animation(semantic_builder)
                print(f"✓ Named segment '{segment_name}' from keyframe {start_index+1} to {end_index+1}")
                print(f"  Saved to {semantic_file_path}")
            else:
                print(f"Failed to name segment '{segment_name}'")
    else:
        print("No keyframes in animation")
    
    return semantic_builder


def handle_play_animation(animation, current_keyframe_index, spot, plant_context, context, meshcat, button_names):
    """Handle playing the animation"""
    if len(animation.animation_keyframes) > 0:
        print("\nPlaying animation with proper timing...")
        
        total_keyframes = len(animation.animation_keyframes)
        
        # Always start by displaying the first keyframe
        current_keyframe_index = 0
        keyframe = animation.animation_keyframes[0]
        print(f"Playing keyframe {1}/{total_keyframes} (Time: {keyframe.time:.2f}s)")
        
        # Update the counter display
        update_keyframe_counter(meshcat, button_names, 0, total_keyframes)
        
        # Load first keyframe into the robot model
        load_joint_angles_from_keyframe(keyframe, spot, plant_context)
        spot.ForcedPublish(context)
        
        # Small initial pause to let the first pose register visually
        time_module.sleep(0.5)
        
        # Then play the remaining keyframes with proper timing
        for i in range(1, total_keyframes):
            prev_keyframe = animation.animation_keyframes[i-1]
            keyframe = animation.animation_keyframes[i]
            
            # Calculate time to wait based on keyframe times
            time_difference = keyframe.time - prev_keyframe.time
            print(f"Playing keyframe {i+1}/{total_keyframes} (Time: {keyframe.time:.2f}s)")
            print(f"  Waiting {time_difference:.2f}s based on keyframe timing")
            
            # Wait the appropriate time
            if time_difference > 0:
                time_module.sleep(time_difference)
            
            # Update the counter display
            update_keyframe_counter(meshcat, button_names, i, total_keyframes)
            
            # Load keyframe into the robot model
            load_joint_angles_from_keyframe(keyframe, spot, plant_context)
            
            # Force visualization update
            spot.ForcedPublish(context)
        
        print("Playback completed")
    else:
        print("No keyframes to play")

    return current_keyframe_index


def handle_stop_animation(is_playing, synced_coordinator, playback_thread):
    """Handle stopping animation playback"""
    if is_playing:
        if synced_coordinator is not None:
            # Stop the playback on real robot
            asyncio.run(synced_coordinator.stop())
        
        # Stop simulation playback
        is_playing = False
        if playback_thread is not None:
            # Wait for playback thread to finish
            playback_thread.join(timeout=0.5)
            playback_thread = None
            
        print("Stopped animation playback")
    else:
        print("No animation playing")
    
    return is_playing, playback_thread


def handle_adjust_timing(animation, semantic_builder):
    """Handle adjusting animation timing"""
    if len(animation.animation_keyframes) > 1:
        print("\nAdjusting animation timing...")
        print("Current keyframe times:")
        keyframe_times = get_keyframe_times(animation)
        for i, time in enumerate(keyframe_times):
            print(f"  Keyframe {i+1}: {time:.2f}s")
        
        # Scale the entire animation by asking for the new end time
        try:
            current_end_time = keyframe_times[-1]
            scale_factor = float(input(f"Current animation length: {current_end_time:.2f}s\nEnter new length in seconds: ")) / current_end_time
            
            # Scale all keyframe times except the first one
            new_times = [0.0] + [time * scale_factor for time in keyframe_times[1:]]
            keyframe_indices = list(range(len(keyframe_times)))
            
            # Adjust the timing
            if adjust_animation_timing(animation, keyframe_indices, new_times):
                # Save the animation
                path_to_animation = Path(get_active_choreo_path(), animation.name)
                save_animation(animation, path_to_animation)
                
                print("\nNew keyframe times:")
                for i, time in enumerate(get_keyframe_times(animation)):
                    print(f"  Keyframe {i+1}: {time:.2f}s")
                
                if semantic_builder is not None:
                    semantic_builder.start_from_animation(animation)
                    save_semantic_animation(semantic_builder)
        except ValueError:
            print("Invalid input, must be a number")
    else:
        print("Need at least 2 keyframes to adjust timing")


def handle_lock_leg(
    leg_type: str,
    button_clicks: int, 
    previous_button_clicks: int,
    button_names: ButtonNames,
    meshcat: Meshcat,
    leg_joint_names: List[str],
    q0_angles: List[float],
    unlocked_legs: List[FootCenterFrame],
    leg_limits: List[List[float]],
    base_joint_slider_names: List[str],
    current_sliders: List[str],
    foot_center_frame: FootCenterFrame
) -> Tuple[int, List[List[float]], List[str], List[float], List[str], List[FootCenterFrame]]:
    """Handle locking/unlocking a leg"""
    previous_button_clicks = button_clicks
    
    if (button_clicks % 2) == 1:
        # Unlock the leg
        leg_limits += leg_limits_base
        base_joint_slider_names.extend(leg_joint_names)
        q0_angles.extend(q0_angles)
        unlocked_legs.append(foot_center_frame)
    else:
        # Lock the leg
        for i in range(len(leg_joint_names)):
            meshcat.DeleteSlider(leg_joint_names[i])
        for c in range(0, 3):
            leg_limits.pop()
            base_joint_slider_names.remove(leg_joint_names[c])
            if leg_joint_names[c] in current_sliders:
                current_sliders.remove(leg_joint_names[c])
            q0_angles.pop()
        unlocked_legs.remove(foot_center_frame)
    
    return previous_button_clicks, leg_limits, base_joint_slider_names, q0_angles, current_sliders, unlocked_legs


def web_animation_loop(with_arm: bool = True) -> None:
    """Creates the meshcat visualizer and IK loop to solve the joint angles as sliders move"""
    # Configure Meshcat for web access with explicit server parameters
    global meshcat_url, spot, plant_context, context
    global meshcat_server  
    
    # Use the standard Meshcat constructor with server parameters
    meshcat = Meshcat()
    meshcat_url = "http://localhost:7000" 
    
    try:
        if hasattr(meshcat, "web_url"):
            meshcat_url = meshcat.web_url()
        elif hasattr(meshcat, "url"):
            meshcat_url = meshcat.url()
        
        print(f"\n--- Meshcat visualization running at: {meshcat_url} ---\n")
    except Exception as e:
        print(f"Could not get Meshcat URL: {e}")
        print(f"Using default URL: {meshcat_url}")


    previous_play_animation_button_clicks = 0
    previous_stop_animation_button_clicks = 0
    previous_adjust_timing_button_clicks = 0
    synced_coordinator = None
    is_playing = False
    playback_thread = None

    keyframe_time = 0
    keyframe_count = 0

    print("\nWould you like to create a new animation or load an existing one?")
    print("1. Create new animation")
    print("2. Load existing animation")
    choice = input("Enter choice (1 or 2): ")
    
    if choice == "2":
        # Get animation name from user
        animation_name = input("Enter animation name to load: ")
        animation_path = Path(get_active_choreo_path(), animation_name, f"{animation_name}.pbtxt")
        
        loaded_animation = load_animation_from_file(animation_path)
        if loaded_animation is not None:
            animation = loaded_animation
            current_keyframe_index = 0
            if len(animation.animation_keyframes) > 0:
                keyframe_time = animation.animation_keyframes[-1].time
            print(f"Loaded animation '{animation.name}' with {len(animation.animation_keyframes)} keyframes")
            
            # Try to load the semantic animation if it exists
            try:
                semantic_animation_path = Path(get_active_choreo_path(), animation_name, f"{animation_name}.pbtxt")
                if semantic_animation_path.exists():
                    semantic_builder = SemanticAnimationBuilder()
                    semantic_builder.load_semantic_animation(semantic_animation_path)
                    print(f"Loaded semantic animation with {len(semantic_builder._semantic_metadata.named_segments)} named segments")
            except Exception as e:
                print(f"Could not load semantic animation: {e}")
                semantic_builder = None
        else:
            # Fall back to creating a new animation if loading fails
            print("Failed to load animation. Creating a new one instead.")
            idle_keyframe = joint_angle_keyframe_to_proto(idle_joint_angles)
            animation = new_animation(idle_keyframe, keyframe_time, keyframe_count)
    else:
        # Setup baseline animation
        idle_keyframe = joint_angle_keyframe_to_proto(idle_joint_angles)
        animation = new_animation(idle_keyframe, keyframe_time, keyframe_count)

    q0_angles = []
    unlocked_legs: List[FootCenterFrame] = []

    button_names = ButtonNames()
    previous_lock_front_left_button_clicks = 0
    previous_lock_front_right_button_clicks = 0
    previous_lock_hind_left_button_clicks = 0
    previous_lock_hind_right_button_clicks = 0

    previous_print_button_clicks = 0

    # State variables for keyframe navigation
    current_keyframe_index = 0
    previous_prev_keyframe_button_clicks = 0
    previous_next_keyframe_button_clicks = 0
    previous_keyframe_info_button_clicks = 0
    previous_update_keyframe_button_clicks = 0
    previous_load_keyframe_button_clicks = 0
    previous_update_time_button_clicks = 0
    
    # State variables for semantic segments
    semantic_builder = None
    previous_name_segment_button_clicks = 0
    
    # State variables for playback
    previous_play_animation_button_clicks = 0
    previous_stop_animation_button_clicks = 0
    previous_adjust_timing_button_clicks = 0
    synced_coordinator = None
    is_playing = False

    try:
        spot, plant_context, arm_joint_slider_names, q_arm, body_pose_sliders, context = (
            setup_default_drake_robot_meshcat(with_arm, meshcat)
        )
        spot_plant = spot.model.get_mutable_multibody_plant()

        from api import spot_animator_connector
        spot_animator_connector.set_robot_references(spot, plant_context, context, meshcat)
        import threading
        spot_animator_connector.spot = spot
        spot_animator_connector.plant_context = plant_context
        spot_animator_connector.context = context
        spot_animator_connector.meshcat = meshcat
        spot_animator_connector.update_robot_function = update_robot_config_from_joint_angles
        spot_animator_connector.active_animation = animation 
        def run_api_server():
            import uvicorn
            uvicorn.run(app, host="0.0.0.0", port=8000)
        api_thread = threading.Thread(target=run_api_server, daemon=True)
        api_thread.start()

        # Sets up the drake UI controls
        front_left_q0_angles, front_right_q0_angles, hind_left_q0_angles, hind_right_q0_angles = setup_ui_buttons(
            meshcat, spot, plant_context, button_names
        )

        # These get updated as buttons are pressed on and off
        current_sliders: List[str] = []
        base_joint_slider_names: List[str] = []
        leg_limits: List[List[float]] = []
        unlocked_legs = []

        previous_save_request_count = 0
        current_save_request_count = 0
        keyframe_time = 1

        body_state = spot.model.get_base_state(plant_context)
        q0_angles = [body_state.front_left_hip_x_q, body_state.front_left_hip_y_q, body_state.front_left_knee_q]

        for i in range(len(base_joint_slider_names)):
            meshcat.AddSlider(
                base_joint_slider_names[i],
                leg_limits[i][0],
                leg_limits[i][1],
                0.01,
                q0_angles[i],
            )
        X_world_body = spot_plant.EvalBodyPoseInWorld(
            plant_context, spot_plant.GetBodyByName("body", spot.model.base_instance)
        )

        animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
        spot.ForcedPublish(context)

        display_save_button(meshcat, button_names)
        
        # Update keyframe counter to show initial state
        if len(animation.animation_keyframes) > 0:
            update_keyframe_counter(meshcat, button_names, current_keyframe_index, len(animation.animation_keyframes))
            
            # Update the segment start/end index sliders to match the animation length
            max_idx = len(animation.animation_keyframes) - 1
            meshcat.DeleteSlider(button_names.segment_start_idx_input)
            meshcat.DeleteSlider(button_names.segment_end_idx_input)
            meshcat.AddSlider(button_names.segment_start_idx_input, 0, max_idx, 1, 0)
            meshcat.AddSlider(button_names.segment_end_idx_input, 0, max_idx, 1, max_idx)
            
            # Set the time slider to match the first keyframe time
            meshcat.SetSliderValue(button_names.keyframe_time_slider_name, animation.animation_keyframes[0].time)

        while True:
            ###
            # UI Interaction Check Start
            ###

            # Meshcat UI interactions have to be manually tracked by whether the number of clicks have changed
            # since the last loop
            lock_front_left_button_clicks = meshcat.GetButtonClicks(button_names.lock_front_left_button_name)
            lock_front_right_button_clicks = meshcat.GetButtonClicks(button_names.lock_front_right_button_name)
            lock_hind_left_button_clicks = meshcat.GetButtonClicks(button_names.lock_hind_left_button_name)
            lock_hind_right_button_clicks = meshcat.GetButtonClicks(button_names.lock_hind_right_button_name)

            print_button_clicks = meshcat.GetButtonClicks(button_names.print_button_name)
            
            # Check keyframe navigation buttons
            prev_keyframe_button_clicks = meshcat.GetButtonClicks(button_names.prev_keyframe_button_name)
            next_keyframe_button_clicks = meshcat.GetButtonClicks(button_names.next_keyframe_button_name)
            keyframe_info_button_clicks = meshcat.GetButtonClicks(button_names.keyframe_info_button_name)
            update_keyframe_button_clicks = meshcat.GetButtonClicks(button_names.update_keyframe_button_name)
            load_keyframe_button_clicks = meshcat.GetButtonClicks(button_names.load_keyframe_button_name)
            
            # Check semantic segment buttons
            name_segment_button_clicks = meshcat.GetButtonClicks(button_names.name_segment_button_name)
            
            # Check playback buttons
            play_animation_button_clicks = meshcat.GetButtonClicks(button_names.play_animation_button_name)
            stop_animation_button_clicks = meshcat.GetButtonClicks(button_names.stop_animation_button_name)
            adjust_timing_button_clicks = meshcat.GetButtonClicks(button_names.adjust_timing_button_name)

            ####
            # Start save pose as keyframe
            ####

            try:
                current_save_request_count = meshcat.GetButtonClicks(button_names.save_button_name)
            except Exception:
                current_save_request_count = current_save_request_count

            if current_save_request_count != previous_save_request_count:
                previous_save_request_count = current_save_request_count
                keyframe_time, keyframe_count, current_keyframe_index = handle_save_keyframe(
                    animation, 
                    animation_keyframe_map, 
                    keyframe_count, 
                    keyframe_time, 
                    meshcat, 
                    button_names, 
                    current_keyframe_index
                )

            ####
            # End save pose as keyframe
            ####
            
            ###
            # KEYFRAME NAVIGATION SECTION
            ###
            
            # Handle previous keyframe button
            if prev_keyframe_button_clicks != previous_prev_keyframe_button_clicks:
                previous_prev_keyframe_button_clicks = prev_keyframe_button_clicks
                result = handle_prev_keyframe(
                    animation, 
                    meshcat, 
                    button_names, 
                    current_keyframe_index, 
                    spot, 
                    plant_context, 
                    context, 
                    body_pose_sliders
                )
                if result[0] is not None:
                    current_keyframe_index = result[0]
                if result[1] is not None:
                    animation_keyframe_map = result[1]
                if result[2] is not None:
                    X_world_body = result[2]
            
            # Handle next keyframe button
            if next_keyframe_button_clicks != previous_next_keyframe_button_clicks:
                previous_next_keyframe_button_clicks = next_keyframe_button_clicks
                result = handle_next_keyframe(
                    animation, 
                    meshcat, 
                    button_names, 
                    current_keyframe_index, 
                    spot, 
                    plant_context, 
                    context, 
                    body_pose_sliders
                )
                if result[0] is not None:
                    current_keyframe_index = result[0]
                if result[1] is not None:
                    animation_keyframe_map = result[1]
                if result[2] is not None:
                    X_world_body = result[2]
            
            # Handle keyframe info button
            if keyframe_info_button_clicks != previous_keyframe_info_button_clicks:
                previous_keyframe_info_button_clicks = keyframe_info_button_clicks
                handle_keyframe_info(animation, current_keyframe_index)
            
            # Handle update keyframe 
            if update_keyframe_button_clicks != previous_update_keyframe_button_clicks:
                previous_update_keyframe_button_clicks = update_keyframe_button_clicks
                handle_update_keyframe(animation, current_keyframe_index, animation_keyframe_map, meshcat, button_names, semantic_builder)
                        
            # Handle load keyframe button
            if load_keyframe_button_clicks != previous_load_keyframe_button_clicks:
                previous_load_keyframe_button_clicks = load_keyframe_button_clicks
                result = handle_load_keyframe(
                    animation, 
                    current_keyframe_index, 
                    spot, 
                    plant_context, 
                    context, 
                    meshcat, 
                    button_names, 
                    body_pose_sliders
                )
                if result is not None:
                    animation_keyframe_map = result
            
            # Handle naming segments
            if name_segment_button_clicks != previous_name_segment_button_clicks:
                previous_name_segment_button_clicks = name_segment_button_clicks
                semantic_builder = handle_name_segment(animation, meshcat, button_names, semantic_builder)
            
            # Handle animation playback
            if play_animation_button_clicks != previous_play_animation_button_clicks:
                previous_play_animation_button_clicks = play_animation_button_clicks
                current_keyframe_index = handle_play_animation(
                    animation,
                    current_keyframe_index,
                    spot,
                    plant_context,
                    context,
                    meshcat,
                    button_names
                )

            # Handle stop playback with improved state tracking
            if stop_animation_button_clicks != previous_stop_animation_button_clicks:
                previous_stop_animation_button_clicks = stop_animation_button_clicks
                is_playing, playback_thread = handle_stop_animation(is_playing, synced_coordinator, playback_thread)

            # Handle adjust timing
            if adjust_timing_button_clicks != previous_adjust_timing_button_clicks:
                previous_adjust_timing_button_clicks = adjust_timing_button_clicks
                handle_adjust_timing(animation, semantic_builder)
            
            ###
            # END KEYFRAME NAVIGATION SECTION
            ###

            ####
            # Start Handle Leg Controls
            ####

            if (lock_front_left_button_clicks != previous_lock_front_left_button_clicks or
                lock_front_right_button_clicks != previous_lock_front_right_button_clicks or
                lock_hind_left_button_clicks != previous_lock_hind_left_button_clicks or
                lock_hind_right_button_clicks != previous_lock_hind_right_button_clicks):

                # Handle front left leg
                if lock_front_left_button_clicks != previous_lock_front_left_button_clicks:
                    result = handle_lock_leg(
                        "front_left",
                        lock_front_left_button_clicks,
                        previous_lock_front_left_button_clicks,
                        button_names,
                        meshcat,
                        button_names.front_left_base_joint_slider_names,
                        front_left_q0_angles,
                        unlocked_legs,
                        leg_limits,
                        base_joint_slider_names,
                        current_sliders,
                        FootCenterFrame.FRONT_LEFT
                    )
                    previous_lock_front_left_button_clicks = result[0]
                    leg_limits = result[1]
                    base_joint_slider_names = result[2]
                    q0_angles = result[3]
                    current_sliders = result[4]
                    unlocked_legs = result[5]

                # Handle front right leg
                if lock_front_right_button_clicks != previous_lock_front_right_button_clicks:
                    result = handle_lock_leg(
                        "front_right",
                        lock_front_right_button_clicks,
                        previous_lock_front_right_button_clicks,
                        button_names,
                        meshcat,
                        button_names.front_right_base_joint_slider_names,
                        front_right_q0_angles,
                        unlocked_legs,
                        leg_limits,
                        base_joint_slider_names,
                        current_sliders,
                        FootCenterFrame.FRONT_RIGHT
                    )
                    previous_lock_front_right_button_clicks = result[0]
                    leg_limits = result[1]
                    base_joint_slider_names = result[2]
                    q0_angles = result[3]
                    current_sliders = result[4]
                    unlocked_legs = result[5]

                # Handle hind left leg
                if lock_hind_left_button_clicks != previous_lock_hind_left_button_clicks:
                    result = handle_lock_leg(
                        "hind_left",
                        lock_hind_left_button_clicks,
                        previous_lock_hind_left_button_clicks,
                        button_names,
                        meshcat,
                        button_names.hind_left_base_joint_slider_names,
                        hind_left_q0_angles,
                        unlocked_legs,
                        leg_limits,
                        base_joint_slider_names,
                        current_sliders,
                        FootCenterFrame.FRONT_LEFT
                    )
                    previous_lock_hind_left_button_clicks = result[0]
                    leg_limits = result[1]
                    base_joint_slider_names = result[2]
                    q0_angles = result[3]
                    current_sliders = result[4]
                    unlocked_legs = result[5]

                # Handle hind right leg
                if lock_hind_right_button_clicks != previous_lock_hind_right_button_clicks:
                    result = handle_lock_leg(
                        "hind_right",
                        lock_hind_right_button_clicks,
                        previous_lock_hind_right_button_clicks,
                        button_names,
                        meshcat,
                        button_names.hind_right_base_joint_slider_names,
                        hind_right_q0_angles,
                        unlocked_legs,
                        leg_limits,
                        base_joint_slider_names,
                        current_sliders,
                        FootCenterFrame.FRONT_RIGHT
                    )
                    previous_lock_hind_right_button_clicks = result[0]
                    leg_limits = result[1]
                    base_joint_slider_names = result[2]
                    q0_angles = result[3]
                    current_sliders = result[4]
                    unlocked_legs = result[5]

                # Adding any new sliders that need to be added
                for i in range(len(base_joint_slider_names)):
                    if base_joint_slider_names[i] in current_sliders:
                        continue
                    meshcat.AddSlider(
                        base_joint_slider_names[i],
                        leg_limits[i][0],
                        leg_limits[i][1],
                        0.01,
                        q0_angles[i],
                    )
                    current_sliders.append(base_joint_slider_names[i])

            ####
            # End Handle Leg Controls
            ####

            ###
            # UI Interaction Check END
            ###

            # READ IN ALL OF THE ARM SLIDERS DIRECTLY
            if spot.model.has_arm:
                animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
                spot.ForcedPublish(context)

            ##### READ IN ALL OF THE LEG SLIDERS DIRECTLY
            body_state = spot.model.get_base_state(plant_context)

            # Extract the idx that each slider name maps to in the position idx
            position_names = spot_plant.GetPositionNames()
            idx_for_position_data = []
            for joint_name in base_joint_slider_names:
                indices = [i for i, x in enumerate(position_names) if joint_name in x]
                idx_for_position_data.append(indices[0])

            base_positions_to_update = spot_plant.GetPositions(plant_context)

            for i, joint_name in enumerate(base_joint_slider_names):
                idx_to_update = idx_for_position_data[i]
                base_positions_to_update[idx_to_update] = meshcat.GetSliderValue(joint_name)

            spot_plant.SetPositions(plant_context, base_positions_to_update)

            ##### END READ IN ALL OF THE LEG SLIDERS DIRECTLY

            # Check if the sliders have changed.
            body_pose_sliders_context = body_pose_sliders.GetMyContextFromRoot(context)

            new_X_world_body = copy(body_pose_sliders.get_output_port().Eval(body_pose_sliders_context))

            if print_button_clicks != previous_print_button_clicks:
                previous_print_button_clicks = print_button_clicks
                print_current_pose_as_keyframe(animation_keyframe_map)

            if X_world_body.IsExactlyEqualTo(new_X_world_body):
                time_module.sleep(0.016)
                continue

            # Compute Ik.
            feet_on_floor = True
            X_world_body = new_X_world_body
            q_ik = spot_body_inverse_kinematics(feet_on_floor, unlocked_legs, X_world_body, spot.model, plant_context)

            animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)

            if q_ik is not None:
                spot_plant.SetPositions(plant_context, q_ik)
                spot.ForcedPublish(context)

        meshcat.DeleteAddedControls()  # type: ignore
    except KeyboardInterrupt:
        pass