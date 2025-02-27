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
    save_button_name = "SAVE POSE AS KEYFRAME"
    lock_front_left_button_name = "Unlock Front Left Leg"
    lock_front_right_button_name = "Unlock Front Right Leg"
    lock_hind_left_button_name = "Unlock Rear Left Leg"
    lock_hind_right_button_name = "Unlock Rear Right Leg"
    print_button_name = "PRINT JOINT ANGLES"
    front_left_base_joint_slider_names = ["fl_hx", "fl_hy", "fl_kn"]
    front_right_base_joint_slider_names = ["fr_hx", "fr_hy", "fr_kn"]
    hind_left_base_joint_slider_names = ["hl_hx", "hl_hy", "hl_kn"]
    hind_right_base_joint_slider_names = ["hr_hx", "hr_hy", "hr_kn"]


def display_save_button(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Add the save pose button to the drake UI"""
    meshcat.AddButton(button_names.save_button_name, "save_button")


def remove_save_button(meshcat: Meshcat, button_names: ButtonNames) -> None:
    """Remove the save pose button to the drake UI"""
    meshcat.DeleteButton(button_names.save_button_name)


def setup_ui_buttons(
    meshcat: Meshcat, spot: Any, plant_context: Any, button_names: ButtonNames
) -> Tuple[List[Any], List[Any], List[Any], List[Any]]:
    """Creates the default UI for the web animator"""
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
    return front_left_q0_angles, front_right_q0_angles, hind_left_q0_angles, hind_right_q0_angles


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
    # Can edit these values (example set x min and max to 0_ to not use given parameters during a run of the code
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
