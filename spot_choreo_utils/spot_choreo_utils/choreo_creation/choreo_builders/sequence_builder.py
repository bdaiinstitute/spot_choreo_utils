# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

import copy
import logging
from typing import Optional, Tuple

from bosdyn.api.spot.choreography_params_pb2 import AnimateParams
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
