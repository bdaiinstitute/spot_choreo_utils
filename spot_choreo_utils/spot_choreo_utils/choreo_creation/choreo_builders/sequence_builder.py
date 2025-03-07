# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy
import logging
from typing import Any, Optional, Tuple

from bosdyn.api.spot.choreography_params_pb2 import (
    AnimateParams,
    BourreeParams,
    Pivot,
    RotateBodyParams,
    SwayParams,
    TwerkParams,
)
from bosdyn.api.spot.choreography_sequence_pb2 import (
    Animation,
    ChoreographySequence,
    MoveParams,
)


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

    def add_rotate_body(
        self,
        start_sec: float,
        duration_sec: float,
        roll: float = 0.05,
        pitch: float = 0.05,
        yaw: float = 0.00,
        return_to_start_pose: bool = True,
    ) -> None:
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

        # Re-frame from time to slices
        slices_per_second = self._sequence.slices_per_minute / 60
        start_slice = int(start_sec * slices_per_second)

        # Calculate the slices to request based on duration
        requested_slices = max(int(duration_sec * slices_per_second), 1)

        # Sanitize inputs
        start_sec = max(start_sec, 0)
        duration_sec = max(start_sec, 0)
        roll = max(min(roll, 0.5), -0.5)
        pitch = max(min(pitch, 0.5), -0.5)
        yaw = max(min(yaw, 0.5), -0.5)

        # Construct the move-specific parameters
        rotate_body_params = RotateBodyParams()
        rotate_body_params.EulerZYXValue.roll.value = roll
        rotate_body_params.EulerZYXValue.pitch.value = pitch
        rotate_body_params.EulerZYXValue.yaw.value = yaw
        rotate_body_params.return_to_start_pose.value = return_to_start_pose

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "rotate_body"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.rotate_body_params.CopyFrom(rotate_body_params)

        # Add to the sequence
        self._sequence.moves.append(move_params)

    def add_sway(
        self,
        start_sec: float,
        duration_sec: float,
        vertical: float = 0.00,
        horizontal: float = 0.05,
        roll: float = 0.00,
        pivot: Any = Pivot.PIVOT_CENTER,  # since Pivot is a non-single protobuf type, we must use Any
        style: Any = SwayParams.SwayStyle.SWAY_STYLE_STANDARD,
        pronounced: float = 0.5,
        hold_zero_axes: bool = False,
    ) -> None:
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

        # Re-frame from time to slices
        slices_per_second = self._sequence.slices_per_minute / 60
        start_slice = int(start_sec * slices_per_second)

        # Calculate the slices to request based on duration
        requested_slices = max(int(duration_sec * slices_per_second), 1)

        # Sanitize inputs
        start_sec = max(start_sec, 0)
        duration_sec = max(start_sec, 0)
        vertical = max(min(roll, 0.2), -0.2)
        horizontal = max(min(horizontal, 0.4), -0.4)
        roll = max(min(roll, 0.2), -0.2)
        pronounced = max(min(pronounced, 1.0), 0.0)

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

        # Add to the sequence
        self._sequence.moves.append(move_params)

    def add_twerk(self, start_sec: float, duration_sec: float, height: float = 0.05) -> None:
        """
        Add a twerk to the sequence

        Args:
            start_sec: absolute time stamp at which point the move should start, in seconds.
            duration_sec: duration of the move, in seconds. This will repeat the move.
            height: target twerk vertical displacement, in meters. Clamped to [0.0, 0.2]. Default is 0.1.

        Returns:
            None
        """

        # Re-frame from time to slices
        slices_per_second = self._sequence.slices_per_minute / 60
        start_slice = int(start_sec * slices_per_second)

        # Calculate the slices to request based on duration
        requested_slices = max(int(duration_sec * slices_per_second), 1)

        # Sanitize inputs
        start_sec = max(start_sec, 0)
        duration_sec = max(start_sec, 0)
        height = max(min(height, 0.2), 0.0)

        # Construct the move-specific parameters
        twerk_params = TwerkParams()
        twerk_params.height.value = height

        # Set up its role within the sequence
        move_params = MoveParams()
        move_params.type = "twerk"
        move_params.start_slice = start_slice
        move_params.requested_slices = requested_slices
        move_params.twerk_params.CopyFrom(twerk_params)

        # Add to the sequence
        self._sequence.moves.append(move_params)

    def add_bourree(
        self,
        start_sec: float,
        duration_sec: float,
        velocity_x: float = 0.00,
        velocity_y: float = 0.00,
        yaw_rate: float = 0.05,
        stance_length: float = 0.00,
    ) -> None:
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

        # Re-frame from time to slices
        slices_per_second = self._sequence.slices_per_minute / 60
        start_slice = int(start_sec * slices_per_second)

        # Calculate the slices to request based on duration
        requested_slices = max(int(duration_sec * slices_per_second), 1)

        # Sanitize inputs
        start_sec = max(start_sec, 0)
        duration_sec = max(start_sec, 0)
        velocity_x = max(min(velocity_x, 0.7), -0.7)
        velocity_y = max(min(velocity_y, 0.5), -0.5)
        yaw_rate = max(min(yaw_rate, 0.7), -0.7)
        stance_length = max(min(stance_length, 0.8), 0.15)

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

        # Add to the sequence
        self._sequence.moves.append(move_params)

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
            return False, "Sequence must contain at least 1 moves"

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
        return True, "success"
