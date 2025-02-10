# Copyright (c) 2024-2025  Boston Dynamics AI Institute LLC. All rights reserved.


import time
from copy import copy
from typing import List, Optional

import numpy as np
import numpy.typing as npt
from pydrake.geometry import Meshcat
from pydrake.geometry.optimization import HPolyhedron, VPolytope
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import (
    InverseKinematics,
)
from pydrake.solvers import Solve
from pydrake.systems.framework import (
    Context,
)
from spot_web_animator.animation_backend.web_animator_drake_ui import (
    ButtonNames,
    display_save_button,
    setup_default_drake_robot_meshcat,
    setup_ui_buttons,
)
from spot_web_animator.animation_backend.web_animator_serializer import (
    joint_angles_to_keyframe,
    new_animation,
    print_as_animation_keyframe,
    print_current_pose_as_keyframe,
    save_pose,
)
from spot_web_animator.systems.spot import (
    FootCenterFrame,
    SpotModel,
)

from spot_choreo_utils.choreo_creation.choreo_builders.animation_proto_utils import (
    joint_angle_keyframe_to_proto,
)

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
    MAXIMUM_BODY_HEIGHT = 0.6  # in meters.
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


def web_animation_loop(with_arm: bool = True) -> None:
    """Creates the meshcat visualizer and IK loop to solve the joint angles as sliders move"""
    meshcat = Meshcat()

    keyframe_time = 0
    keyframe_count = 0

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

    try:
        spot, plant_context, arm_joint_slider_names, q_arm, body_pose_sliders, context = (
            setup_default_drake_robot_meshcat(with_arm, meshcat)
        )
        spot_plant = spot.model.get_mutable_multibody_plant()

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
        q0_angles = [body_state.fl_hx_q, body_state.fl_hy_q, body_state.fl_kn_q]

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

            ####
            # Start save pose as keyframe
            ####

            try:
                current_save_request_count = meshcat.GetButtonClicks(button_names.save_button_name)
            except Exception:
                current_save_request_count = current_save_request_count

            if current_save_request_count != previous_save_request_count:
                previous_save_request_count = current_save_request_count
                keyframe = joint_angles_to_keyframe(animation_keyframe_map)
                keyframe_time += 1
                save_pose(keyframe, keyframe_count, keyframe_time, animation)

            ####
            # End save pose as keyframe
            ####

            ####
            # Start Handle Leg Controls
            ####

            if (
                lock_front_left_button_clicks != previous_lock_front_left_button_clicks
                or lock_front_right_button_clicks != previous_lock_front_right_button_clicks
                or lock_hind_left_button_clicks != previous_lock_hind_left_button_clicks
                or lock_hind_right_button_clicks != previous_lock_hind_right_button_clicks
            ):
                # FRONT LEFT LEG##
                if lock_front_left_button_clicks != previous_lock_front_left_button_clicks:
                    previous_lock_front_left_button_clicks = lock_front_left_button_clicks
                    if (lock_front_left_button_clicks % 2) == 1:
                        leg_limits += leg_limits_base
                        base_joint_slider_names.extend(button_names.front_left_base_joint_slider_names)
                        q0_angles.extend(front_left_q0_angles)
                        body_state = spot.model.get_base_state(plant_context)
                        unlocked_legs.append(FootCenterFrame.FRONT_LEFT)
                    else:
                        for i in range(len(button_names.front_left_base_joint_slider_names)):
                            meshcat.DeleteSlider(button_names.front_left_base_joint_slider_names[i])
                        for c in range(0, 3):
                            leg_limits.pop()
                            base_joint_slider_names.remove(button_names.front_left_base_joint_slider_names[c])
                            current_sliders.remove(button_names.front_left_base_joint_slider_names[c])
                            q0_angles.remove(front_left_q0_angles[c])
                        unlocked_legs.remove(FootCenterFrame.FRONT_LEFT)

                ##FRONT RIGHT LEG##
                if lock_front_right_button_clicks != previous_lock_front_right_button_clicks:
                    previous_lock_front_right_button_clicks = lock_front_right_button_clicks
                    if (lock_front_right_button_clicks % 2) == 1:
                        leg_limits += leg_limits_base
                        base_joint_slider_names.extend(button_names.front_right_base_joint_slider_names)
                        q0_angles.extend(front_right_q0_angles)
                        body_state = spot.model.get_base_state(plant_context)
                        unlocked_legs.append(FootCenterFrame.FRONT_RIGHT)
                    else:
                        for i in range(len(button_names.front_right_base_joint_slider_names)):
                            meshcat.DeleteSlider(button_names.front_right_base_joint_slider_names[i])
                        for c in range(0, 3):
                            leg_limits.pop()
                            base_joint_slider_names.remove(button_names.front_right_base_joint_slider_names[c])
                            current_sliders.remove(button_names.front_right_base_joint_slider_names[c])
                            q0_angles.remove(front_right_q0_angles[c])
                        unlocked_legs.remove(FootCenterFrame.FRONT_RIGHT)

                # HIND LEFT LEG##
                if lock_hind_left_button_clicks != previous_lock_hind_left_button_clicks:
                    previous_lock_hind_left_button_clicks = lock_hind_left_button_clicks
                    if (lock_hind_left_button_clicks % 2) == 1:
                        leg_limits += leg_limits_base
                        base_joint_slider_names.extend(button_names.hind_left_base_joint_slider_names)
                        q0_angles.extend(hind_left_q0_angles)
                        body_state = spot.model.get_base_state(plant_context)
                        unlocked_legs.append(FootCenterFrame.FRONT_LEFT)
                    else:
                        for i in range(len(button_names.hind_left_base_joint_slider_names)):
                            meshcat.DeleteSlider(button_names.hind_left_base_joint_slider_names[i])
                        for c in range(0, 3):
                            leg_limits.pop()
                            base_joint_slider_names.remove(button_names.hind_left_base_joint_slider_names[c])
                            current_sliders.remove(button_names.hind_left_base_joint_slider_names[c])
                            q0_angles.remove(hind_left_q0_angles[c])
                        unlocked_legs.remove(FootCenterFrame.FRONT_LEFT)

                ##HIND RIGHT LEG##
                if lock_hind_right_button_clicks != previous_lock_hind_right_button_clicks:
                    previous_lock_hind_right_button_clicks = lock_hind_right_button_clicks
                    if (lock_hind_right_button_clicks % 2) == 1:
                        leg_limits += leg_limits_base
                        base_joint_slider_names.extend(button_names.hind_right_base_joint_slider_names)
                        q0_angles.extend(hind_right_q0_angles)
                        body_state = spot.model.get_base_state(plant_context)
                        unlocked_legs.append(FootCenterFrame.FRONT_RIGHT)
                    else:
                        for i in range(len(button_names.hind_right_base_joint_slider_names)):
                            meshcat.DeleteSlider(button_names.hind_right_base_joint_slider_names[i])
                        for c in range(0, 3):
                            leg_limits.pop()
                            base_joint_slider_names.remove(button_names.hind_right_base_joint_slider_names[c])
                            current_sliders.remove(button_names.hind_right_base_joint_slider_names[c])
                            q0_angles.remove(hind_right_q0_angles[c])
                        unlocked_legs.remove(FootCenterFrame.FRONT_RIGHT)

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
                # Read the arm sliders and set the arm positions in the plant_context.
                for i in range(spot.model.num_arm_joints()):
                    q_arm[i] = meshcat.GetSliderValue(arm_joint_slider_names[i])

                animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)
                spot_plant.SetPositions(plant_context, spot.model.arm_instance, q_arm)
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
                time.sleep(1e-3)
                continue

            # Compute Ik.
            feet_on_floor = True
            X_world_body = new_X_world_body
            q_ik = spot_body_inverse_kinematics(feet_on_floor, unlocked_legs, X_world_body, spot.model, plant_context)

            animation_keyframe_map = print_as_animation_keyframe(spot_plant, spot, plant_context, X_world_body)

            if q_ik is not None:
                t = time.localtime()
                time.strftime("%H:%M:%S", t)
                spot_plant.SetPositions(plant_context, q_ik)
                spot.ForcedPublish(context)

        meshcat.DeleteAddedControls()  # type: ignore
    except KeyboardInterrupt:
        pass
