# Copyright (c) 2025 Robotics and AI Institute LLC dba RAI Institute. All rights reserved.

# Copyright (c) 2024-2025  Boston Dynamics AI Institute LLC. All rights reserved.

from collections import namedtuple
from typing import Any, List, Tuple

import numpy as np
from pydrake.geometry import Meshcat
from pydrake.systems.framework import (
    Context,
    DiagramBuilder,
)
from pydrake.visualization import MeshcatPoseSliders
from spot_web_animator.systems.spot import (
    MockSpot,
)

RpyXyz = namedtuple("RpyXyz", ("roll", "pitch", "yaw", "x", "y", "z"))


class ButtonNames:
    # Robot control buttons
    save_button_name = "SAVE POSE AS KEYFRAME"
    lock_front_left_button_name = "Unlock Front Left Leg"
    lock_front_right_button_name = "Unlock Front Right Leg"
    lock_hind_left_button_name = "Unlock Rear Left Leg"
    lock_hind_right_button_name = "Unlock Rear Right Leg"
    print_button_name = "PRINT JOINT ANGLES"

    # Leg joint slider names
    front_left_base_joint_slider_names = ["fl_hx", "fl_hy", "fl_kn"]
    front_right_base_joint_slider_names = ["fr_hx", "fr_hy", "fr_kn"]
    hind_left_base_joint_slider_names = ["hl_hx", "hl_hy", "hl_kn"]
    hind_right_base_joint_slider_names = ["hr_hx", "hr_hy", "hr_kn"]

    # Keyframe navigation and editing buttons
    prev_keyframe_button_name = "◀ PREV KEYFRAME"
    next_keyframe_button_name = "NEXT KEYFRAME ▶"
    keyframe_info_button_name = "SHOW KEYFRAME INFO"
    update_keyframe_button_name = "UPDATE KEYFRAME"
    load_keyframe_button_name = "LOAD KEYFRAME"
    keyframe_section_header = "--- KEYFRAME EDITOR ---"
    keyframe_counter_name = "KEYFRAME: 0/0"
    keyframe_time_slider_name = "Keyframe Time (s)"

    # Semantic animation section
    semantic_section_header = "--- SEMANTIC SEGMENTS ---"
    name_segment_button_name = "NAME SEGMENT"
    segment_name_input = "Segment Name:"
    segment_start_idx_input = "Start Index:"
    segment_end_idx_input = "End Index:"

    # Playback section
    playback_section_header = "--- ANIMATION PLAYBACK ---"
    play_animation_button_name = "▶ PLAY ANIMATION"
    stop_animation_button_name = "■ STOP ANIMATION"
    adjust_timing_button_name = "ADJUST ALL TIMING"


def setup_keyframe_navigation_ui(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Sets up UI for keyframe navigation and editing"""
    # Add a section header to visually separate keyframe navigation buttons
    meshcat.AddButton(button_names.keyframe_section_header, "keyframe_nav_header")

    # Add keyframe counter display
    meshcat.AddButton(button_names.keyframe_counter_name, "keyframe_counter")

    # Add navigation buttons
    meshcat.AddButton(button_names.prev_keyframe_button_name, "prev_keyframe")
    meshcat.GetButtonClicks(button_names.prev_keyframe_button_name)

    meshcat.AddButton(button_names.next_keyframe_button_name, "next_keyframe")
    meshcat.GetButtonClicks(button_names.next_keyframe_button_name)

    # Add info and editing buttons
    meshcat.AddButton(button_names.keyframe_info_button_name, "keyframe_info")
    meshcat.GetButtonClicks(button_names.keyframe_info_button_name)

    meshcat.AddButton(button_names.load_keyframe_button_name, "load_keyframe")
    meshcat.GetButtonClicks(button_names.load_keyframe_button_name)

    # Add time adjustment slider
    meshcat.AddSlider(button_names.keyframe_time_slider_name, 0.0, 60.0, 0.1, 0.0)

    # Update the button text to indicate it handles both pose and time updates
    button_names.update_keyframe_button_name = "UPDATE KEYFRAME & TIME"
    meshcat.AddButton(button_names.update_keyframe_button_name, "update_keyframe")
    meshcat.GetButtonClicks(button_names.update_keyframe_button_name)


def setup_semantic_segment_ui(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Sets up UI for semantic segment naming"""
    # Add section header
    meshcat.AddButton(button_names.semantic_section_header, "semantic_header")

    # Use sliders for start and end indices
    meshcat.AddSlider(button_names.segment_start_idx_input, 0, 100, 1, 0)
    meshcat.AddSlider(button_names.segment_end_idx_input, 0, 100, 1, 0)

    # Add button to name segment
    meshcat.AddButton(button_names.name_segment_button_name, "name_segment")
    meshcat.GetButtonClicks(button_names.name_segment_button_name)


def setup_playback_ui(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Sets up UI for animation playback"""
    # Add section header
    meshcat.AddButton(button_names.playback_section_header, "playback_header")

    # Add play and stop buttons
    meshcat.AddButton(button_names.play_animation_button_name, "play_animation")
    meshcat.GetButtonClicks(button_names.play_animation_button_name)

    meshcat.AddButton(button_names.stop_animation_button_name, "stop_animation")
    meshcat.GetButtonClicks(button_names.stop_animation_button_name)

    # Add timing adjustment button
    meshcat.AddButton(button_names.adjust_timing_button_name, "adjust_timing")
    meshcat.GetButtonClicks(button_names.adjust_timing_button_name)


def setup_ui_buttons(
    meshcat: Meshcat, spot: Any, plant_context: Any, button_names: ButtonNames
) -> Tuple[List[Any], List[Any], List[Any], List[Any]]:
    """Creates the default UI for the web animator"""
    # Add a section header to visually separate robot control buttons
    meshcat.AddButton("--- ROBOT CONTROLS ---", "robot_controls_header")

    ##Buttons to lock and unlock the legs
    body_state = spot.model.get_base_state(plant_context)

    meshcat.AddButton(button_names.lock_front_left_button_name, "fl Lock")
    meshcat.GetButtonClicks(button_names.lock_front_left_button_name)
    front_left_q0_angles = [body_state.front_left_hip_x_q, body_state.front_left_hip_y_q, body_state.front_left_knee_q]

    meshcat.AddButton(button_names.lock_front_right_button_name, "fr Lock")
    meshcat.GetButtonClicks(button_names.lock_front_right_button_name)
    front_right_q0_angles = [
        body_state.front_right_hip_x_q,
        body_state.front_right_hip_y_q,
        body_state.front_right_knee_q,
    ]

    meshcat.AddButton(button_names.lock_hind_left_button_name, "fl Lock")
    meshcat.GetButtonClicks(button_names.lock_hind_left_button_name)
    hind_left_q0_angles = [body_state.rear_left_hip_x_q, body_state.rear_left_hip_y_q, body_state.rear_left_knee_q]

    meshcat.AddButton(button_names.lock_hind_right_button_name, "fr Lock")
    meshcat.GetButtonClicks(button_names.lock_hind_right_button_name)
    hind_right_q0_angles = [body_state.rear_right_hip_x_q, body_state.rear_right_hip_y_q, body_state.rear_right_knee_q]

    meshcat.AddButton(button_names.print_button_name, "print keyframe")
    meshcat.GetButtonClicks(button_names.print_button_name)

    # Add keyframe navigation UI
    setup_keyframe_navigation_ui(meshcat, button_names)

    # Add semantic segment UI
    setup_semantic_segment_ui(meshcat, button_names)

    # Add playback UI
    setup_playback_ui(meshcat, button_names)

    return front_left_q0_angles, front_right_q0_angles, hind_left_q0_angles, hind_right_q0_angles


def update_keyframe_counter(meshcat: Meshcat, button_names: ButtonNames, current: int, total: int) -> None:
    """Updates the keyframe counter display"""
    meshcat.DeleteButton(button_names.keyframe_counter_name)
    button_names.keyframe_counter_name = f"KEYFRAME: {current+1}/{total}"
    meshcat.AddButton(button_names.keyframe_counter_name, "keyframe_counter")


def display_save_button(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Add the save pose button to the drake UI"""
    meshcat.AddButton(button_names.save_button_name, "save_button")


def remove_save_button(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Remove the save pose button to the drake UI"""
    meshcat.DeleteButton(button_names.save_button_name)


def setup_default_drake_robot_meshcat(
    with_arm: bool, meshcat: Meshcat
) -> tuple[MockSpot, Context, list[str], Any, Any, Any]:
    """Builds the basic plant and sets the robot in a default pose"""

    meshcat.Delete()
    meshcat.DeleteAddedControls()

    spot = MockSpot(has_arm=with_arm)
    spot_plant = spot.model.get_mutable_multibody_plant()

    diagram_builder = DiagramBuilder()

    # Set up teleop widgets.
    # Can edit these values to change the limits of the sliders
    body_pose_sliders = diagram_builder.AddSystem(
        MeshcatPoseSliders(
            meshcat,
            lower_limit=RpyXyz(roll=-0.5 * np.pi, pitch=-np.pi / 2, yaw=-1, x=-0.3, y=-0.5, z=0.1),
            upper_limit=RpyXyz(roll=0.5 * np.pi, pitch=np.pi / 2, yaw=1, x=0.3, y=0.5, z=0.7),
        )
    )
    spot.Finalize(meshcat, diagram_builder)
    context = spot.CreateDefaultContext()
    plant_context = spot_plant.GetMyMutableContextFromRoot(context)

    # Set the initial configuration.
    arm_joint_slider_names = []

    if spot.model.has_arm:
        # Add joint sliders for each joint.
        q0_arm = spot.model.get_arm_state(plant_context)[: spot.model.num_arm_positions()]
        for i in range(spot.model.num_arm_joints()):
            arm_joint_slider_names.append(spot.model.ArmStateView.get_fields()[i].replace("_q", ""))
            meshcat.AddSlider(
                arm_joint_slider_names[-1],
                spot_plant.GetPositionLowerLimits()[spot.model.num_base_positions() :][i],
                spot_plant.GetPositionUpperLimits()[spot.model.num_base_positions() :][i],
                0.01,
                q0_arm[i],
            )

    # Set the initial configuration.
    spot.model.set_default_base_configuration()
    if spot.model.has_arm:
        spot.model.set_default_arm_configuration()
        spot.model.set_default_gripper_configuration()

    if spot.model.has_arm:
        q_arm = spot.model.get_arm_state(plant_context)[: spot.model.num_arm_positions()]
    X_world_body = spot_plant.EvalBodyPoseInWorld(
        plant_context, spot_plant.GetBodyByName("body", spot.model.base_instance)
    )
    body_pose_sliders.SetPose(X_world_body)
    return (spot, plant_context, arm_joint_slider_names, q_arm, body_pose_sliders, context)
