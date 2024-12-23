# Copyright (c) 2023-2025 Boston Dynamics AI Institute LLC. All rights reserved.

from bosdyn.api.spot.choreography_sequence_pb2 import (
    MoveParams,
)

from spot_choreo_utils.choreo_creation.choreo_builders.sequence_builder import SequenceBuilder

FIXED_LENGTH_MOVES = just_adjust_slice_list = [
    "turn_2step",
    "arm_move",
    "chicken_head",
    "butt_circle",
    "jump",
    "body_hold",
    "rotate_body_sharp",
    "step",
    "twerk",
    "stand_to_kneel",
    "kneel_circles",
    "kneel_clap",
    "kneel_to_stand",
    "animation",
    "stow",
    "goto",
    "sit",
    "bourree",
]


def adjust_sequence_fidelity(builder: SequenceBuilder, precision_increase: int) -> None:
    """Increase the slice precision of the sequence"""
    builder.set_slices_per_minute(builder.slices_per_minute * precision_increase)
    for move in builder.raw_moves:
        if move.requested_slices:
            move.requested_slices *= precision_increase
        if move.start_slice:
            move.start_slice *= precision_increase


def get_move_start_slice_for_time(builder: SequenceBuilder, time: float) -> int:
    """Calculate the slice that starts a move prior to the requested time"""
    slices_per_minute = builder.slices_per_minute
    target_slice = int(time * float(slices_per_minute) / 60.0)
    best_idx = 0
    for move in builder.raw_moves:
        if move.start_slice > target_slice:
            best_idx = best_idx - 1 if best_idx > 0 else best_idx
            return builder.raw_moves[best_idx].start_slice
        best_idx += 1
    return -1


def adjust_move_start(move: MoveParams, adjust_by_slices: int) -> None:
    """
    Adjust the start point of a move based on specific paramaters that need adjusted for each move type
    Will throw error if adjustment isn't implemented yet
    """
    if move.type in FIXED_LENGTH_MOVES:
        move.start_slice += adjust_by_slices
    else:
        raise NotImplementedError(f"Move type {move.type} not implemented")


def adjust_move_length(move: MoveParams, adjust_by_slices: int) -> None:
    """
    Adjust the duration of a move based on specific paramaters that need adjusted for each move type
    Will throw error if adjustment isn't implemented yet
    """
    if move.type in FIXED_LENGTH_MOVES:
        move.requested_slices += adjust_by_slices
    elif move.type == "butt_circle":
        current_circle_amount = move.butt_circle_params.beats_per_circle.value * move.requested_slices
        move.requested_slices += adjust_by_slices
        move.butt_circle_params.beats_per_circle.value = current_circle_amount / move.requested_slices
    elif move.type == "kneel_circles":
        current_circle_amount = move.kneel_circle_params.beats_per_circle.value * move.requested_slices
        move.requested_slices += adjust_by_slices
        move.kneel_circle_params.beats_per_circle.value = int(current_circle_amount / move.requested_slices)
    else:
        raise NotImplementedError(f"Move type {move.type} not implemented")


def adjust_sequence_by_time(
    builder: SequenceBuilder, adjustment_start_idx: int, adjustment_end_idx: int, time_to_adjust: float
) -> None:
    """Adjust the start slice of move by idx, adjust length of move before,
    then adjust the start slice of all remaining moves"""
    slices_per_second = builder.slices_per_minute / 60
    adjust_by_slices = int(slices_per_second * time_to_adjust)
    adjust_sequence_by_slices(builder, adjustment_start_idx, adjustment_end_idx, adjust_by_slices)


def adjust_sequence_by_slices(
    builder: SequenceBuilder, adjustment_start_idx: int, adjustment_end_idx: int, adjust_by_slices: int
) -> None:
    """Adjust the start slice of move by idx, adjust length of move before,
    then adjust the start slice of all remaining moves"""

    for idx in range(adjustment_start_idx, adjustment_end_idx):
        adjust_move_start(builder.raw_moves[idx], adjust_by_slices)

    # Adjust the move one before
    # Extend the previous move
    if adjustment_start_idx > 0:
        adjust_move_length(builder.raw_moves[adjustment_start_idx - 1], adjust_by_slices)

    # The last move that's part of this adjusted set should be shortened
    if adjust_by_slices > 0 and adjustment_end_idx < len(builder.raw_moves):
        move = builder.raw_moves[adjustment_end_idx - 1]
        adjust_move_length(move, -adjust_by_slices)


def remove_zero_length_moves(builder: SequenceBuilder) -> None:
    idxs_to_remove = []
    for idx, move in enumerate(builder.raw_moves):
        if move.requested_slices <= 0:
            idxs_to_remove.append(idx)

    for idx in reversed(idxs_to_remove):
        del builder.raw_moves[idx]
