# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy
import logging
from typing import Any, Dict, List, Optional, Tuple

from bosdyn.api.spot.choreography_params_pb2 import (
    AnimateParams,
    ArmMoveFrame,
    BourreeParams,
    ButtCircleParams,
    ChickenHeadParams,
    Easing,
    FidgetStandParams,
    Pivot,
    RotateBodyParams,
    SwayParams,
    TurnParams,
    TwerkParams,
    WorkspaceArmMoveParams,
)
from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    ChoreographySequence,
    MoveParams,
)
from google.protobuf.wrappers_pb2 import DoubleValue

PARAM_NAME_TO_BOUNDS = {
    "start_slice": (0.0, float("inf")),
    "requested_slices": (1.0, float("inf")),
    "rotate_body_roll": (-0.5, 0.5),
    "rotate_body_pitch": (-0.5, 0.5),
    "rotate_body_yaw": (-0.5, 0.5),
    "sway_vertical": (-0.2, 0.2),
    "sway_horizontal": (-0.2, 0.2),
    "sway_roll": (-0.2, 0.2),
    "sway_pronounced": (0.0, 1.0),
    "twerk_height": (0.0, 0.2),
    "bourree_velocity_x": (-0.7, 0.7),
    "bourree_velocity_y": (-0.5, 0.5),
    "bourree_yaw_rate": (-0.7, 0.7),
    "bourree_stance_length": (0.15, 0.8),
    "turn_motion_x": (-0.8, 0.8),
    "turn_motion_y": (-0.4, 0.4),
    "turn_absolute_motion_x": (-0.8, 0.8),
    "turn_absolute_motion_y": (-0.4, 0.4),
    "turn_yaw": (-1.6, 1.6),
    "turn_absolute_yaw": (-1.6, 1.6),
    "turn_swing_height": (0.0, 0.3),
    "turn_swing_velocity": (0.0, 3.0),
    "butt_circle_radius": (0.0, 0.2),
    "butt_circle_beats_per_circle": (0.1, 8.0),
    "butt_circle_number_of_circles": (0.0, 100.0),
    "butt_circle_starting_angle": (-3.14, 3.14),
    "rotate_body_rotation_roll": (-0.50, 0.50),
    "rotate_body_rotation_pitch": (-0.50, 0.50),
    "rotate_body_rotation_yaw": (-0.50, 0.50),
    "workspace_arm_move_rotation_roll": (-3.14, 3.14),
    "workspace_arm_move_rotation_pitch": (-3.14, 3.14),
    "workspace_arm_move_rotation_yaw": (-3.14, 3.14),
    "workspace_arm_move_translation_x": (-1.50, 1.50),
    "workspace_arm_move_translation_y": (-1.50, 1.50),
    "workspace_arm_move_translation_z": (-1.50, 1.50),
    "chicken_head_bob_magnitude_x": (-0.50, 0.50),
    "chicken_head_bob_magnitude_y": (-0.50, 0.50),
    "chicken_head_bob_magnitude_z": (-0.50, 0.50),
    "fidget_stand_min_gaze_pitch": (-1.20, 0.00),
    "fidget_stand_max_gaze_pitch": (0.00, 1.20),
    "fidget_stand_gaze_mean_period": (0.5, 60.0),
    "fidget_stand_gaze_center_cfp_x": (0.20, 0.80),
    "fidget_stand_gaze_center_cfp_y": (-0.20, 0.20),
    "fidget_stand_gaze_center_cfp_z": (-0.50, 1.10),
    "fidget_stand_shift_mean_period": (0.5, 60.0),
    "fidget_stand_shift_max_transition_time": (0.2, 3.0),
    "fidget_stand_breath_min_z": (0.00, 0.04),
    "fidget_stand_breath_max_z": (0.00, 0.04),
    "fidget_stand_breath_max_period": (0.5, 10.0),
    "fidget_stand_leg_gesture_mean_period": (1.0, 60.0),
    "fidget_stand_gaze_slew_rate": (0.1, 10.0),
    "fidget_stand_gaze_position_generation_gain_x": (0.00, 1.00),
    "fidget_stand_gaze_position_generation_gain_y": (0.00, 1.00),
    "fidget_stand_gaze_position_generation_gain_z": (0.00, 1.00),
    "fidget_stand_gaze_roll_generation_gain": (0.00, 2.00),
}


class SequenceBuilder:
    """Convenient wrapper for building, editing, and validating choreography sequences"""

    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        self._sequence = ChoreographySequence()
        self._logger = logger

    def start_from_empty(self, name: str, slices_per_minute: int = 6000) -> None:
        """Build a sequence procedurally move by move"""
        self._sequence = ChoreographySequence()
        self._sequence.name = name
        self._sequence.slices_per_minute = slices_per_minute

    def start_from_sequence(self, sequence: ChoreographySequence) -> None:
        """Modify an existing sequence with builder helper functions"""
        self._sequence = copy.deepcopy(sequence)

    @property
    def slices_per_minute(self) -> int:
        """Get the playback slices per minute"""
        return self._sequence.slices_per_minute

    @property
    def raw_moves(self) -> list[MoveParams]:
        """Access the underlying moves in the sequence"""
        return self._sequence.moves

    @property
    def move_types(self) -> list[str]:
        """Returns a list of all moves in the sequence"""
        return [move.type for move in self.raw_moves]

    def set_slices_per_minute(self, slices_per_minute: int) -> None:
        """Update the slices per minute"""
        self._sequence.slices_per_minute = slices_per_minute

    def add_animation(self, animation: Animation, start_time: float) -> None:
        """Add an animation to the sequence"""
        if animation is None:
            if self._logger is not None:
                self._logger.error("Can't add animation None to sequence")
            return
        if not animation.animation_keyframes:
            if self._logger is not None:
                self._logger.error("Animation is empty, can't add to sequence")
            return

        # Re-frame from time to slices
        slices_per_second = self._sequence.slices_per_minute / 60
        start_slice = int(start_time * slices_per_second)

        # Calculate the slices to request based on animation length
        animation_length = animation.animation_keyframes[-1].time
        if animation_length:
            # Requested slices must be at least 1
            requested_slices = max(int(animation_length * slices_per_second), 1)
        else:
            requested_slices = len(animation.animation_keyframes)

        # Add the animation definition to the sequence
        animation_params = AnimateParams()
        animation_params.animation_name = animation.name

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "animation"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.animate_params.CopyFrom(animation_params)

        # Add to the sequence
        self._sequence.moves.append(move_params)

    def add_moves(self, moves_list: List[Dict[str, Any]]) -> None:
        """
        Add a set of moves to the sequence

        Args:
            moves_list: list of per-move dictionaries, each containing the move type
                and keyword args specific to that move type.
                i.e.    moves_list = [
                            {
                                "type": "sway",
                                "start_sec": 0.0,
                                "duration_sec": 0.5,
                                "horizontal": 0.161
                            },
                            {
                                "type": "twerk",
                                "start_slice": 48,
                                "requested_slices": 24,
                                "height": 0.1
                            },
                        ]

        Returns:
            None
        """

        prev_end_slice = 0
        for move in moves_list:
            move_type = move.pop("type", None)
            move_adder = getattr(self, "add_" + move_type, None)

            # If this move lacks a specified start time, make it start immediately after the previous move in the list
            no_start_time = "start_sec" not in move and "start_slice" not in move
            if no_start_time:
                move["start_slice"] = prev_end_slice
                # TODO: refactor here and elsewhere so we stick to slices by default prob

            if move_adder is not None:
                move_adder(**move)
                prev_end_slice = move["start_slice"] + move["requested_slices"]
            else:
                raise ValueError(f"Unsupported move type: {move_type}")

    def validate_move(self, move_params: MoveParams) -> Tuple[bool, str]:
        if move_params.type == "animation":
            return (
                True,
                "Animation move is validated during build(), skipping validation.",
            )

        if move_params.type == "stow" or move_params.type == "unstow":
            return (
                True,
                "This move has no params, skipping validation.",
            )

        ## Need to handle certain moves manually, as BD's naming occasionally overlaps or is inconsistent
        if move_params.type == "rotate_body_sharp":  # because rotate_body_sharp shares rotate_body's RotateBodyParams
            move_specific_validator: Optional[Any] = self.validate_rotate_body_sharp
            move_specific_params: Optional[Any] = move_params.rotate_body_params
        elif (
            move_params.type == "turn_2step"
        ):  # because BD's turn_2step move uses TurnParams, not Turn2StepParams as we might expect
            move_specific_validator = self.validate_turn
            move_specific_params = move_params.turn_params
        else:
            move_specific_validator = getattr(self, "validate_" + move_params.type, None)
            move_specific_params = getattr(move_params, move_params.type + "_params", None)

        if move_specific_validator is None or move_specific_params is None:
            return (
                False,
                (
                    f"Move of type {move_params.type} either lacks a validator function, or is provided without a"
                    f" {move_params.type}_params member"
                ),
            )

        return move_specific_validator(move_specific_params)

    def _clamp_param(self, name: str, value_pb: DoubleValue) -> DoubleValue:
        low_bound, high_bound = PARAM_NAME_TO_BOUNDS[name]
        warn_str = None
        if value_pb.value < low_bound:
            warn_str = f"Value {value_pb.value} for param {name} has been clamped to lower bound {low_bound}."
            value_pb = DoubleValue(value=low_bound)
        elif value_pb.value > high_bound:
            warn_str = f"Value {value_pb.value} for param {name} has been clamped to upper bound {high_bound}."
            value_pb = DoubleValue(value=high_bound)

        if warn_str is not None and self._logger is not None:
            self._logger.warning(warn_str)
        return value_pb

    def validate_rotate_body(self, params: RotateBodyParams) -> Tuple[bool, str]:
        params.rotation.roll.CopyFrom(self._clamp_param("rotate_body_roll", params.rotation.roll))
        params.rotation.pitch.CopyFrom(self._clamp_param("rotate_body_pitch", params.rotation.pitch))
        params.rotation.yaw.CopyFrom(self._clamp_param("rotate_body_yaw", params.rotation.yaw))
        return True, "Success"

    def validate_rotate_body_sharp(self, params: RotateBodyParams) -> Tuple[bool, str]:
        return self.validate_rotate_body(params)

    def validate_sway(self, params: SwayParams) -> Tuple[bool, str]:
        params.vertical.CopyFrom(self._clamp_param("sway_vertical", params.vertical))
        params.horizontal.CopyFrom(self._clamp_param("sway_horizontal", params.horizontal))
        params.roll.CopyFrom(self._clamp_param("sway_roll", params.roll))
        if params.style not in SwayParams.SwayStyle.values():
            params.style = SwayParams.SWAY_STYLE_STANDARD
        if params.style > 1:
            params.pronounced.CopyFrom(self._clamp_param("sway_pronounced", params.pronounced))
        return True, "Success"

    def validate_twerk(self, params: TwerkParams) -> Tuple[bool, str]:
        params.height.CopyFrom(self._clamp_param("twerk_height", params.height))
        return True, "Success"

    def validate_bourree(self, params: BourreeParams) -> Tuple[bool, str]:
        params.velocity.x.CopyFrom(self._clamp_param("bourree_velocity_x", params.velocity.x))
        params.velocity.y.CopyFrom(self._clamp_param("bourree_velocity_y", params.velocity.y))
        params.yaw_rate.CopyFrom(self._clamp_param("bourree_yaw_rate", params.yaw_rate))
        params.stance_length.CopyFrom(self._clamp_param("bourree_stance_length", params.stance_length))
        return True, "Success"

    def validate_turn(self, params: TurnParams) -> Tuple[bool, str]:
        params.motion.x.CopyFrom(self._clamp_param("turn_motion_x", params.motion.x))
        params.motion.y.CopyFrom(self._clamp_param("turn_motion_y", params.motion.y))
        params.absolute_motion.x.CopyFrom(self._clamp_param("turn_absolute_motion_x", params.absolute_motion.x))
        params.absolute_motion.y.CopyFrom(self._clamp_param("turn_absolute_motion_y", params.absolute_motion.y))
        params.yaw.CopyFrom(self._clamp_param("turn_yaw", params.yaw))
        params.absolute_yaw.CopyFrom(self._clamp_param("turn_absolute_yaw", params.absolute_yaw))
        params.swing_height.CopyFrom(self._clamp_param("turn_swing_height", params.swing_height))
        params.swing_velocity.CopyFrom(self._clamp_param("turn_swing_velocity", params.swing_velocity))
        return True, "Success"

    def validate_fidget_stand(self, params: FidgetStandParams) -> Tuple[bool, str]:
        if params.preset not in FidgetStandParams.FidgetPreset.values():
            params.preset = FidgetStandParams.FidgetPreset.PRESET_CUSTOM
        params.min_gaze_pitch.CopyFrom(self._clamp_param("fidget_stand_min_gaze_pitch", params.min_gaze_pitch))
        params.max_gaze_pitch.CopyFrom(self._clamp_param("fidget_stand_max_gaze_pitch", params.max_gaze_pitch))
        params.gaze_mean_period.CopyFrom(self._clamp_param("fidget_stand_gaze_mean_period", params.gaze_mean_period))
        params.gaze_center_cfp.x.CopyFrom(self._clamp_param("fidget_stand_gaze_center_cfp_x", params.gaze_center_cfp.x))
        params.gaze_center_cfp.y.CopyFrom(self._clamp_param("fidget_stand_gaze_center_cfp_y", params.gaze_center_cfp.y))
        params.gaze_center_cfp.z.CopyFrom(self._clamp_param("fidget_stand_gaze_center_cfp_z", params.gaze_center_cfp.z))
        params.shift_mean_period.CopyFrom(self._clamp_param("fidget_stand_shift_mean_period", params.shift_mean_period))
        params.shift_max_transition_time.CopyFrom(
            self._clamp_param("fidget_stand_shift_max_transition_time", params.shift_max_transition_time)
        )
        params.breath_min_z.CopyFrom(self._clamp_param("fidget_stand_breath_min_z", params.breath_min_z))
        params.breath_max_z.CopyFrom(self._clamp_param("fidget_stand_breath_max_z", params.breath_max_z))
        params.breath_max_period.CopyFrom(self._clamp_param("fidget_stand_breath_max_period", params.breath_max_period))
        params.leg_gesture_mean_period.CopyFrom(
            self._clamp_param("fidget_stand_leg_gesture_mean_period", params.leg_gesture_mean_period)
        )
        params.gaze_slew_rate.CopyFrom(self._clamp_param("fidget_stand_gaze_slew_rate", params.gaze_slew_rate))
        params.gaze_position_generation_gain.x.CopyFrom(
            self._clamp_param("fidget_stand_gaze_position_generation_gain_x", params.gaze_position_generation_gain.x)
        )
        params.gaze_position_generation_gain.y.CopyFrom(
            self._clamp_param("fidget_stand_gaze_position_generation_gain_y", params.gaze_position_generation_gain.y)
        )
        params.gaze_position_generation_gain.z.CopyFrom(
            self._clamp_param("fidget_stand_gaze_position_generation_gain_z", params.gaze_position_generation_gain.z)
        )
        params.gaze_roll_generation_gain.CopyFrom(
            self._clamp_param("fidget_stand_gaze_roll_generation_gain", params.gaze_roll_generation_gain)
        )
        return True, "Success"

    def validate_workspace_arm_move(self, params: WorkspaceArmMoveParams) -> Tuple[bool, str]:
        params.rotation.roll.CopyFrom(self._clamp_param("workspace_arm_move_rotation_roll", params.rotation.roll))
        params.rotation.pitch.CopyFrom(self._clamp_param("workspace_arm_move_rotation_pitch", params.rotation.pitch))
        params.rotation.yaw.CopyFrom(self._clamp_param("workspace_arm_move_rotation_yaw", params.rotation.yaw))
        params.translation.x.CopyFrom(self._clamp_param("workspace_arm_move_translation_x", params.translation.x))
        params.translation.y.CopyFrom(self._clamp_param("workspace_arm_move_translation_y", params.translation.y))
        params.translation.z.CopyFrom(self._clamp_param("workspace_arm_move_translation_z", params.translation.z))
        if params.frame not in ArmMoveFrame.values():
            params.frame = WorkspaceArmMoveParams.ARM_MOVE_FRAME_CENTER_OF_FOOTPRINT
        if params.easing not in Easing.values():
            params.easing = Easing.EASING_LINEAR
        return True, "Success"

    def validate_chicken_head(self, params: ChickenHeadParams) -> Tuple[bool, str]:
        params.bob_magnitude.x.CopyFrom(self._clamp_param("chicken_head_bob_magnitude_x", params.bob_magnitude.x))
        params.bob_magnitude.y.CopyFrom(self._clamp_param("chicken_head_bob_magnitude_y", params.bob_magnitude.y))
        params.bob_magnitude.z.CopyFrom(self._clamp_param("chicken_head_bob_magnitude_z", params.bob_magnitude.z))
        return True, "Success"

    def validate_butt_circle(self, params: ButtCircleParams) -> Tuple[bool, str]:
        params.radius.CopyFrom(self._clamp_param("butt_circle_radius", params.radius))
        params.beats_per_circle.CopyFrom(self._clamp_param("butt_circle_beats_per_circle", params.beats_per_circle))
        params.number_of_circles.CopyFrom(self._clamp_param("butt_circle_number_of_circles", params.number_of_circles))
        params.starting_angle.CopyFrom(self._clamp_param("butt_circle_starting_angle", params.starting_angle))
        return True, "Success"

    def add_rotate_body(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        roll: float = 0.05,
        pitch: float = 0.05,
        yaw: float = 0.00,
        return_to_start_pose: bool = True,
    ) -> Tuple[bool, str]:
        """
        Add a rotate_body to the sequence

        Args:
            start_sec: absolute time stamp at which point the move should start, in seconds.
            duration_sec: duration of the move, in seconds. This will repeat the move.
            roll: target body roll, in radians. Clamped to [-0.5, 0.5]. Default is 0.05.
            pitch: target body pitch, in radians. Clamped to [-0.5, 0.5]. Default is 0.05.
            yaw: target body yaw, in radians. Clamped to [-0.5, 0.5]. Default is 0.00.
            return_to_start_pose: will return body to the previous pose by the end of this move.

        Returns:
            None
        """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        rotate_body_params = RotateBodyParams()
        rotate_body_params.rotation.roll.value = roll
        rotate_body_params.rotation.pitch.value = pitch
        rotate_body_params.rotation.yaw.value = yaw
        rotate_body_params.return_to_start_pose.value = return_to_start_pose

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "rotate_body"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.rotate_body_params.CopyFrom(rotate_body_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_sway(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        vertical: float = 0.00,
        horizontal: float = 0.05,
        roll: float = 0.00,
        pivot: Any = Pivot.PIVOT_CENTER,  # since Pivot is a non-single protobuf type, we must use Any
        style: Any = SwayParams.SwayStyle.SWAY_STYLE_STANDARD,
        pronounced: float = 0.5,
        hold_zero_axes: bool = False,
    ) -> Tuple[bool, str]:
        """
        Add a sway to the sequence

        Args:
            start_sec: absolute time stamp at which point the move should start,
                in seconds.
            duration_sec: duration of the move, in seconds. This will repeat the move.
            vertical: target body vertical displacement, in meters. Clamped to
                [-0.2, 0.2]. Default is 0.00.
            horizontal: target body horizontal displacement, in meters. Clamped to
                [-0.4, 0.4]. Default is 0.05.
            roll: target body roll, in radians. Clamped to [-0.2, 0.2]. Default is 0.00.
            pivot: which portion of the body remains stationary, of type Pivot enum.
            style: the velocity profile of the move, of type SwayParams.SwayStyle enum.
            pronounced: how exaggerated the style is. The closer to 0, the closer to
                standard style. Clamped to [0.0, 1.0]. Default is 0.5.
            hold_zero_axes: whether to maintain the previous position and rotation for
                any axes set to 0. Default is False.

        Returns:
            None
        """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        sway_params = SwayParams()
        sway_params.vertical.value = vertical
        sway_params.horizontal.value = horizontal
        sway_params.roll.value = roll
        sway_params.pivot = pivot
        sway_params.style = style
        sway_params.pronounced.value = pronounced
        sway_params.hold_zero_axes.value = hold_zero_axes

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "sway"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.sway_params.CopyFrom(sway_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_twerk(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        height: float = 0.05,
    ) -> Tuple[bool, str]:
        """
        Add a twerk to the sequence

        Args:
            start_sec: absolute time stamp at which point the move should start, in seconds.
            duration_sec: duration of the move, in seconds. This will repeat the move.
            height: target twerk vertical displacement, in meters. Clamped to [0.0, 0.2]. Default is 0.1.

        Returns:
            None
        """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        twerk_params = TwerkParams()
        twerk_params.height.value = height

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "twerk"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.twerk_params.CopyFrom(twerk_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_bourree(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        velocity_x: float = 0.00,
        velocity_y: float = 0.00,
        yaw_rate: float = 0.05,
        stance_length: float = 0.60,
    ) -> Tuple[bool, str]:
        """
        Add a bourree to the sequence

        Args:
            start_sec: absolute time stamp at which point the move should start, in seconds.
            duration_sec: duration of the move, in seconds. This will repeat the move.
            velocity_x: target x (fore/aft) velocity, in m/s. Clamped to [-0.7, 0.7]. Default is 0.0.
            velocity_y: target y (left/right) velocity, in m/s. Clamped to [-0.5, 0.5]. Default is 0.0.
            yaw_rate: target yaw rate, in rad/s. Clamped to [-0.7, 0.7]. Default is 0.05.
            stance_length: distance between front and hind legs. Clamped to [0.15, 0.80]. Default is 0.60.

        Returns:
            None
        """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        bourree_params = BourreeParams()
        bourree_params.velocity.x.value = velocity_x
        bourree_params.velocity.y.value = velocity_y
        bourree_params.yaw_rate.value = yaw_rate
        bourree_params.stance_length.value = stance_length

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "bourree"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.bourree_params.CopyFrom(bourree_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_turn(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        motion_is_absolute: bool = False,
        motion_x: float = 0.0,
        motion_y: float = 0.0,
        absolute_motion_x: float = 0.0,
        absolute_motion_y: float = 0.0,
        yaw_is_absolute: bool = False,
        yaw: float = 0.80,
        absolute_yaw: float = 0.00,
        swing_height: float = 0.00,
        swing_velocity: float = 0.00,
    ) -> Tuple[bool, str]:
        """

        NOTE: a "turn" move is the same as a "turn_2step" move, which are two different names BD uses for the same move
        """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        turn_params = TurnParams()
        turn_params.motion_is_absolute.value = motion_is_absolute
        if motion_is_absolute:
            turn_params.absolute_motion.x.value = absolute_motion_y
            turn_params.absolute_motion.y.value = absolute_motion_y
        else:
            turn_params.motion.x.value = motion_x
            turn_params.motion.y.value = motion_y
        turn_params.yaw_is_absolute.value = yaw_is_absolute
        if yaw_is_absolute:
            turn_params.absolute_yaw.value = yaw
        else:
            turn_params.yaw.value = yaw
        turn_params.swing_height.value = swing_height
        turn_params.swing_velocity.value = swing_velocity

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "turn_2step"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.turn_params.CopyFrom(turn_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_butt_circle(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        radius: float = 0.10,
        beats_per_circle: float = 2.00,
        number_of_circles: float = 0.00,
        pivot: Any = Pivot.PIVOT_FRONT,
        clockwise: bool = True,
        starting_angle: float = 0.00,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        butt_circle_params = ButtCircleParams()
        butt_circle_params.radius.value = radius
        butt_circle_params.beats_per_circle.value = beats_per_circle
        butt_circle_params.number_of_circles.value = number_of_circles
        butt_circle_params.pivot = pivot
        butt_circle_params.clockwise.value = clockwise
        butt_circle_params.starting_angle.value = starting_angle

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "butt_circle"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.butt_circle_params.CopyFrom(butt_circle_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_rotate_body_sharp(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        roll: float = 0.00,
        pitch: float = 0.00,
        yaw: float = 0.00,
        return_to_start_pose: bool = True,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        rotate_body_sharp_params = RotateBodyParams()
        rotate_body_sharp_params.rotation.roll.value = roll
        rotate_body_sharp_params.rotation.pitch.value = pitch
        rotate_body_sharp_params.rotation.yaw.value = yaw
        rotate_body_sharp_params.return_to_start_pose.value = return_to_start_pose

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "rotate_body_sharp"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.rotate_body_params.CopyFrom(rotate_body_sharp_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_fidget_stand(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        preset: Any = FidgetStandParams.FidgetPreset,
        min_gaze_pitch: float = -1.00,
        max_gaze_pitch: float = 1.00,
        gaze_mean_period: float = 8.00,
        gaze_center_cfp_x: float = 0.50,
        gaze_center_cfp_y: float = 0.00,
        gaze_center_cfp_z: float = 0.75,
        shift_mean_period: float = 10.00,
        shift_max_transition_time: float = 1.20,
        breath_min_z: float = 0.01,
        breath_max_z: float = 0.02,
        breath_max_period: float = 4.00,
        leg_gesture_mean_period: float = 10.00,
        gaze_slew_rate: float = 0.50,
        gaze_position_generation_gain_x: float = 0.30,
        gaze_position_generation_gain_y: float = 0.30,
        gaze_position_generation_gain_z: float = 0.30,
        gaze_roll_generation_gain: float = 0.60,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters

        fidget_stand_params = FidgetStandParams()
        fidget_stand_params.preset = preset
        fidget_stand_params.min_gaze_pitch.value = min_gaze_pitch
        fidget_stand_params.max_gaze_pitch.value = max_gaze_pitch
        fidget_stand_params.gaze_mean_period.value = gaze_mean_period
        fidget_stand_params.gaze_center_cfp.x.value = gaze_center_cfp_x
        fidget_stand_params.gaze_center_cfp.y.value = gaze_center_cfp_y
        fidget_stand_params.gaze_center_cfp.z.value = gaze_center_cfp_z
        fidget_stand_params.shift_mean_period.value = shift_mean_period
        fidget_stand_params.shift_max_transition_time.value = shift_max_transition_time
        fidget_stand_params.breath_min_z.value = breath_min_z
        fidget_stand_params.breath_max_z.value = breath_max_z
        fidget_stand_params.breath_max_period.value = breath_max_period
        fidget_stand_params.leg_gesture_mean_period.value = leg_gesture_mean_period
        fidget_stand_params.gaze_slew_rate.value = gaze_slew_rate
        fidget_stand_params.gaze_position_generation_gain.x.value = gaze_position_generation_gain_x
        fidget_stand_params.gaze_position_generation_gain.y.value = gaze_position_generation_gain_y
        fidget_stand_params.gaze_position_generation_gain.z.value = gaze_position_generation_gain_z
        fidget_stand_params.gaze_roll_generation_gain.value = gaze_roll_generation_gain

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "fidget_stand"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.fidget_stand_params.CopyFrom(fidget_stand_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_stow(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "stow"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_unstow(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "unstow"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_workspace_arm_move(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        roll: float = 0.05,
        pitch: float = 0.05,
        yaw: float = 0.05,
        translation_x: float = 0.00,
        translation_y: float = 0.00,
        translation_z: float = 0.00,
        absolute: bool = False,
        frame: Any = ArmMoveFrame.ARM_MOVE_FRAME_CENTER_OF_FOOTPRINT,
        easing: Any = Easing.EASING_CUBIC_IN_OUT,
        dance_frame_id: int = 0,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        workspace_arm_move_params = WorkspaceArmMoveParams()
        workspace_arm_move_params.rotation.roll.value = roll
        workspace_arm_move_params.rotation.pitch.value = pitch
        workspace_arm_move_params.rotation.yaw.value = yaw
        workspace_arm_move_params.translation.x.value = translation_x
        workspace_arm_move_params.translation.y.value = translation_y
        workspace_arm_move_params.translation.z.value = translation_z
        workspace_arm_move_params.absolute.value = absolute
        workspace_arm_move_params.frame = frame
        workspace_arm_move_params.easing = easing
        workspace_arm_move_params.dance_frame_id.value = dance_frame_id

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "workspace_arm_move"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.workspace_arm_move_params.CopyFrom(workspace_arm_move_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def add_chicken_head(
        self,
        start_sec: Optional[float] = None,
        duration_sec: Optional[float] = None,
        start_slice: Optional[float] = None,
        requested_slices: Optional[float] = None,
        bob_magnitude_x: float = 0.00,
        bob_magnitude_y: float = 0.00,
        bob_magnitude_z: float = 0.00,
        beats_per_cycle: int = 2,
        follow: bool = False,
    ) -> Tuple[bool, str]:
        """ """

        if start_sec is not None and duration_sec is not None:
            # Re-frame from time to slices
            slices_per_second = self._sequence.slices_per_minute / 60
            start_slice = int(start_sec * slices_per_second)
            # Calculate the slices to request based on duration
            requested_slices = max(int(duration_sec * slices_per_second), 1)
        elif start_slice is None or requested_slices is None:
            return False, "Either start_sec and duration_sec or start_slice and requested_slices must be provided."

        # Construct the move-specific parameters
        chicken_head_params = ChickenHeadParams()
        chicken_head_params.bob_magnitude.x.value = bob_magnitude_x
        chicken_head_params.bob_magnitude.y.value = bob_magnitude_y
        chicken_head_params.bob_magnitude.z.value = bob_magnitude_z
        chicken_head_params.beats_per_cycle.value = beats_per_cycle
        chicken_head_params.follow.value = follow

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "chicken_head"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.chicken_head_params.CopyFrom(chicken_head_params)

        res, msg = self.validate_move(move_params)
        if not res:
            fail_str = f"Failed to validate move: {msg}. Not adding this move to sequence."
            if self._logger is not None:
                self._logger.warning(fail_str)
            return False, fail_str

        # Add to the sequence
        self._sequence.moves.append(move_params)
        return True, "success"

    def build(self) -> ChoreographySequence:
        """
        Validate and return a protobuf representing the final sequence.

        Returns None if the sequence is invalid and would fail executing on robot
        """
        res, msg = self.validate()
        if not res:
            if self._logger is not None:
                self._logger.error(f"Failed to build sequence: {msg}")
            return None
        return self._sequence

    def move_param_duration_limits(self) -> Tuple[int, int]:
        """Get the slice duration limits specified by the robot"""
        return (1, 2147483647)

    def validate(self) -> Tuple[bool, str]:
        """Offline validator that matches reasons sequences fail to exceute on robot"""
        if not self._sequence.name:
            return False, "Sequence has no name"

        if not self._sequence.slices_per_minute:
            return False, "Must specify slices per minute"

        if not self._sequence.moves:
            return False, "Sequence must contain at least 1 move"

        for idx, move in enumerate(self._sequence.moves):
            if move.start_slice < 0:
                return False, "Start slice must be positive"

            min_slices, max_slices = self.move_param_duration_limits()
            if move.requested_slices > max_slices or move.requested_slices < min_slices:
                return (
                    False,
                    (
                        f"Move at idx {idx} requested slices out of range. Reqested {move.requested_slices} but range"
                        f" is {min_slices}-{max_slices}"
                    ),
                )
            res, msg = self.validate_move(move)
            if not res:
                return False, f"Move failed to validate: {msg}"

        return True, "success"
